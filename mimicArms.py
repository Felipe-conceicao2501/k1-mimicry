"""Arm mimicry: webcam -> MediaPipe Pose -> `move_arm` over WebSocket to a Booster K1.

STATUS: BETA. The client side is complete; the robot-side service does not implement
the arm protocol yet (see "Prerequisites" below). Not verified end-to-end.

v2 (2026-08-22). Rewritten after reviewing the platform's service inventory: the K1 has
NO hand joint (gripper / dexterous hand are optional C++ SDK accessories, not installed
on this robot), so this script never handled fingers, and it now also drops any notion
of "hand orientation" (wrist roll/pitch/yaw stay fixed at 0). The focus is only the ARM
(shoulder + elbow) on both sides. Legs stay out of scope.

The SDK path for moving the arm is `B1LocoClient.MoveHandEndEffector` (Cartesian
position of the arm's end effector, not "hand" in the sense of fingers), preceded by
`SwitchHandEndEffectorControlMode`. This script speaks the protocol that a robot-side
service is expected to expose for that:

  {"op": "switch_arm_control", "enable": true}   once at session start
  {"op": "move_arm", "hand": 0|1, "x":.., "y":.., "z":.., "time_ms": ..}   per frame
  {"op": "switch_arm_control", "enable": false}  once at exit

Prerequisites that are still missing before this works end to end:
  1) The robot-side WebSocket service does NOT implement "switch_arm_control" /
     "move_arm" yet; they exist only in this script's protocol for now. The matching
     robot-side change must be deployed before testing.
  2) That service currently listens only on 127.0.0.1 on the robot, not on 0.0.0.0, so
     it cannot be reached directly from the laptop. Open an SSH tunnel first:
       ssh -N -L 8765:127.0.0.1:8765 <user>@<robot-ip>
     and use ROBOT_IP=127.0.0.1 here (no robot-side change is needed for the tunnel).

Tracking (what changed in the vision part, to make it more ACCURATE):
  v1 used the 2D wrist-to-shoulder distance (in normalized pixels) as a proxy for "how
  far the arm is extended forward", with a fixed normalization constant (0.35). Two
  real problems: (a) that distance grows the same whether the arm extends UP, SIDEWAYS
  or FORWARD, so it cannot tell direction, only "how far the wrist is from the
  shoulder"; (b) the fixed constant breaks when the person is nearer to or farther from
  the webcam (the same arm extension becomes very different numbers).

  v2 uses the ELBOW ANGLE (shoulder-elbow-wrist) as the extension signal. It is
  invariant to camera scale/distance (an angle, not a pixel distance) and measures a
  real joint instead of raw position. Bent elbow (~30-40 deg) = arm retracted; straight
  elbow (~170-180 deg) = arm extended. For the wrist's lateral (Y) and height (Z), the
  2D position relative to the shoulder is still the right signal, but it is now
  normalized by the person's OWN arm length (shoulder-elbow distance in pixels) instead
  of a fixed constant, so it self-calibrates to the person's size/distance.

Mapping to the target pose (x=forward, y=lateral, z=height, meters, robot torso frame),
sent via "move_arm". Still an APPROXIMATION (not real IK, no true metric depth), but
more robust than v1. The camera display is mirrored ("mirror mode": raising your right
arm raises the robot's right arm, from the point of view of someone facing the robot).

Safety envelope (Cartesian, not published by the platform; this is our own single
source of truth, and the robot clamps again on its side as defense in depth):
  x: 0.15 to 0.40 (forward)   y: -0.40 to 0.40 (lateral, + is left)   z: -0.15 to 0.35 (height)

Usage:
  ROBOT_IP=127.0.0.1 python3 mimicArms.py   # with the SSH tunnel above already running

ESC closes the window and sends "switch_arm_control" with enable=false before exiting
(turns off the arm end-effector control mode on the robot).
"""
from __future__ import annotations

import json
import math
import os
import socket
import struct
import sys
import time

import cv2
import mediapipe as mp

# ---- safety envelope (our own SSOT; the robot clamps again on its side) ----
X_MIN, X_MAX = 0.15, 0.40
Y_MIN, Y_MAX = -0.40, 0.40
Z_MIN, Z_MAX = -0.15, 0.35
SEND_HZ = 8.0
TIME_MS = 200  # motion duration on the robot, slightly longer than the send interval
VISIBILITY_MIN = 0.6  # only send a command if shoulder/elbow/wrist are clearly visible

# ---- arm extension from elbow angle (degrees) ----
ELBOW_BEND_MIN = 35.0   # elbow fully bent -> extension 0.0
ELBOW_BEND_MAX = 170.0  # arm straight -> extension 1.0

# ---- lateral/height normalization by the person's own arm length (auto-scale) ----
# Empirical factor: how many "shoulder-elbow distances" the wrist reaches sideways / in
# height at the limit of the envelope (arm wide open to the side or raised).
REACH_SCALE = 2.2

# SDK return codes that deserve a small backoff: 429 = called too frequently, 409 = state
# transition in progress. We do not retry with the SAME parameters on 400 (out of
# range), we only log it.
RC_HINTS = {
    429: "calling too fast, delaying sends",
    409: "robot state conflict (transition in progress), retrying shortly",
    400: "invalid parameter/range - do NOT repeat unchanged",
    502: "robot state transition failure",
    100: "robot timeout (network/service) - check the robot-side motion service",
}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------- minimal WebSocket client (stdlib, RFC 6455) ----------
class WSClient:
    def __init__(self, host: str, port: int):
        self.sock = socket.create_connection((host, port), timeout=5)
        key = "k1-mimic-arms-key=="
        req = (
            f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
            f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
        )
        self.sock.sendall(req.encode())
        resp = self.sock.recv(4096)
        if b"101" not in resp:
            raise RuntimeError(f"WebSocket handshake failed: {resp!r}")
        self.sock.settimeout(2.0)

    def send(self, msg: dict) -> dict | None:
        data = json.dumps(msg).encode()
        mask = os.urandom(4)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        n = len(data)
        if n < 126:
            hdr = struct.pack(">BB", 0x81, 0x80 | n)
        else:
            hdr = struct.pack(">BBH", 0x81, 0x80 | 126, n)
        self.sock.sendall(hdr + mask + masked)
        try:
            hdr2 = self.sock.recv(2)
            if not hdr2:
                return None
            ln = hdr2[1] & 0x7F
            if ln == 126:
                ln = struct.unpack(">H", self.sock.recv(2))[0]
            payload = b""
            while len(payload) < ln:
                chunk = self.sock.recv(ln - len(payload))
                if not chunk:
                    break
                payload += chunk
            return json.loads(payload.decode())
        except socket.timeout:
            return None

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def _report_rc(label: str, reply: dict | None) -> None:
    """Log the return code in a readable way instead of dumping the raw dict (RPC
    return codes used to be discarded without handling)."""
    if reply is None:
        print(f"[mimic] {label}: no reply (timeout)")
        return
    if reply.get("ok"):
        return
    rc = reply.get("rc")
    hint = RC_HINTS.get(rc, reply.get("error", "unknown error"))
    print(f"[mimic] {label}: rc={rc} -> {hint}")


def _elbow_angle_deg(shoulder, elbow, wrist) -> float:
    """Elbow angle in degrees, between the vectors (elbow->shoulder) and
    (elbow->wrist). ~180 = arm straight, ~30-40 = arm fully bent. Invariant to camera
    scale/distance (an angle, not a pixel distance)."""
    v1x, v1y = shoulder.x - elbow.x, shoulder.y - elbow.y
    v2x, v2y = wrist.x - elbow.x, wrist.y - elbow.y
    n1 = math.hypot(v1x, v1y)
    n2 = math.hypot(v2x, v2y)
    if n1 < 1e-6 or n2 < 1e-6:
        return 90.0
    cos_a = clamp((v1x * v2x + v1y * v2y) / (n1 * n2), -1.0, 1.0)
    return math.degrees(math.acos(cos_a))


def arm_target(shoulder, elbow, wrist) -> tuple[float, float, float]:
    """Map the arm pose (shoulder/elbow/wrist, normalized 0..1 image coordinates) to a
    target pose (x, y, z in meters, robot torso frame).

    x (forward): arm extension via the elbow angle, scale-invariant.
    y (lateral), z (height): wrist position relative to the shoulder, normalized by the
    person's own shoulder-elbow length (self-calibrates to camera distance)."""
    angle = _elbow_angle_deg(shoulder, elbow, wrist)
    extension = clamp((angle - ELBOW_BEND_MIN) / (ELBOW_BEND_MAX - ELBOW_BEND_MIN), 0.0, 1.0)
    x = X_MIN + extension * (X_MAX - X_MIN)

    dx_img = wrist.x - shoulder.x   # + = wrist to the right of the image
    dy_img = shoulder.y - wrist.y   # + = wrist ABOVE the shoulder (image y grows downward)
    arm_scale = max(math.hypot(shoulder.x - elbow.x, shoulder.y - elbow.y), 1e-3)

    # Lateral (y): the camera is mirrored on display. dx_img > 0 in the ORIGINAL
    # (non-mirrored) MediaPipe frame = wrist to the RIGHT of the person facing the
    # camera, which must become NEGATIVE lateral (+y is left, the robot's convention).
    y_norm = clamp(-dx_img / (arm_scale * REACH_SCALE), -1.0, 1.0)
    y = clamp(y_norm * Y_MAX, Y_MIN, Y_MAX)

    z_norm = clamp(dy_img / (arm_scale * REACH_SCALE), -1.0, 1.0)
    z = Z_MIN + (z_norm + 1.0) / 2.0 * (Z_MAX - Z_MIN)

    return (round(x, 3), round(y, 3), round(z, 3))


def main():
    host = os.environ.get("ROBOT_IP")
    if not host:
        print("Set ROBOT_IP (e.g. ROBOT_IP=127.0.0.1 python3 mimicArms.py, with the "
              "SSH tunnel to the robot already running; see the docstring at the top)")
        sys.exit(1)
    port = int(os.environ.get("ROBOT_MOTION_PORT", "8765"))

    print(f"[mimic] connecting to ws://{host}:{port} ...")
    ws = WSClient(host, port)
    print("[mimic] connected. Enabling arm control (switch_arm_control)...")
    _report_rc("switch_arm_control(true)", ws.send({"op": "switch_arm_control", "enable": True}))
    print("[mimic] Press ESC in the webcam window to quit.")

    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils

    cap = cv2.VideoCapture(0)
    last_send = 0.0
    send_interval = 1.0 / SEND_HZ
    backoff_until = 0.0  # timestamp; while now < this, skip sending (429 backoff)

    LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST = 11, 13, 15
    RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST = 12, 14, 16

    try:
        with mp_pose.Pose(model_complexity=1, min_detection_confidence=0.6,
                          min_tracking_confidence=0.6) as pose:
            while cap.isOpened():
                ok, frame = cap.read()
                if not ok:
                    print("[mimic] empty frame, skipping")
                    continue

                frame.flags.writeable = False
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = pose.process(rgb)
                frame.flags.writeable = True

                now = time.monotonic()
                due = (now - last_send) >= send_interval and now >= backoff_until

                if results.pose_landmarks:
                    mp_drawing.draw_landmarks(
                        frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

                    if due:
                        lm = results.pose_landmarks.landmark
                        last_send = now
                        # Arm 0 = the robot's left arm (on the LEFT of someone facing
                        # the robot) = the person's RIGHT side in the camera
                        # (MediaPipe's non-mirrored frame). So use the RIGHT_*
                        # landmarks for hand=0 and vice versa, to get a true mirror:
                        # you raise your right arm, the robot raises ITS left arm,
                        # which is on your right side when it faces you.
                        triples = [
                            (0, lm[RIGHT_SHOULDER], lm[RIGHT_ELBOW], lm[RIGHT_WRIST]),
                            (1, lm[LEFT_SHOULDER], lm[LEFT_ELBOW], lm[LEFT_WRIST]),
                        ]
                        for hand, shoulder, elbow, wrist in triples:
                            if (shoulder.visibility < VISIBILITY_MIN
                                    or elbow.visibility < VISIBILITY_MIN
                                    or wrist.visibility < VISIBILITY_MIN):
                                continue
                            x, y, z = arm_target(shoulder, elbow, wrist)
                            reply = ws.send({
                                "op": "move_arm", "hand": hand,
                                "x": x, "y": y, "z": z,
                                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                                "time_ms": TIME_MS,
                            })
                            label = "L" if hand == 0 else "R"
                            print(f"[mimic] arm={label} x={x} y={y} z={z} -> {reply}")
                            if reply is not None and not reply.get("ok", False):
                                _report_rc(f"move_arm({label})", reply)
                                if reply.get("rc") == 429:
                                    backoff_until = now + 0.5

                cv2.imshow("Mimic Arms (ESC to quit)", cv2.flip(frame, 1))
                if cv2.waitKey(1) & 0xFF == 27:
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[mimic] disabling arm control (switch_arm_control)...")
        try:
            _report_rc("switch_arm_control(false)",
                       ws.send({"op": "switch_arm_control", "enable": False}))
        except Exception as e:  # noqa: BLE001
            print(f"[mimic] switch_arm_control(false) failed (ignoring): {e}")
        ws.close()
        print("[mimic] stopped.")


if __name__ == "__main__":
    main()
