"""Head mimicry: webcam -> MediaPipe Face Mesh + solvePnP -> `rotate_head` over
WebSocket to a Booster K1.

v2 (2026-08-12): v1 used only the nose position relative to the shoulder (MediaPipe
Pose). That is a WEAK proxy for head rotation: turning the head without turning the
body moves the nose very little in the image, even for a large turn. Result: "micro
movements" that did not follow real head turns (observed in a live test).

v2 estimates the real 3D head ORIENTATION: six Face Mesh landmarks + a generic 3D face
model + cv2.solvePnP (the classic head-pose-estimation technique used in OpenCV/dlib
tutorials). This measures rotation directly instead of a translation proxy, so it is
far more sensitive to actual head turns.

Commands are sent to a robot-side WebSocket motion service (not included in this
repository) using the `rotate_head` op, which already existed and was verified on the
robot, so no robot-side code change was needed.

Usage:
  ROBOT_IP=<robot-ip> python3 mimicHead.py

ESC closes the window and re-centers the head (yaw=0, pitch=0) before exiting.

Sign/direction: solvePnP does not guarantee that the yaw/pitch signs match the robot's
convention out of the box. YAW_SIGN / PITCH_SIGN below exist so they can be flipped
quickly if the robot turns the wrong way on the first real test (this was seen and
adjusted live).
"""
from __future__ import annotations

import json
import os
import select
import socket
import struct
import sys
import time

import cv2
import mediapipe as mp
import numpy as np

# Conservative envelope (inside what was verified with the SDK's example CLI: yaw up to
# ~0.785, pitch -0.3..1.0), tightened here for smooth mimicry.
YAW_MAX = 0.7
PITCH_MIN, PITCH_MAX = -0.25, 0.45
SEND_HZ = 8.0
YAW_SIGN = 1.0
PITCH_SIGN = 1.0

# solvePnP stability: with only 6 points and no continuity between frames, the pose can
# "jump" to an ambiguous alternative solution on any frame (seen live: pitch=79 out of
# nowhere). Two defenses: (1) use the previous frame's estimate as the initial guess
# (useExtrinsicGuess) so the solver stays near the previous solution instead of jumping
# to another one; (2) EMA + a maximum per-update jump on the final angles, filtering
# whatever still gets through.
SMOOTH_ALPHA = 0.35       # EMA: lower = smoother but more lag
MAX_DELTA_DEG = 15.0      # max accepted jump per update (~every 1/SEND_HZ s)

# Generic 3D face model (arbitrary units, the same reference used in classic OpenCV/dlib
# head-pose tutorials). Order must match FACE_LANDMARK_IDX below.
MODEL_POINTS = np.array([
    (0.0, 0.0, 0.0),          # nose tip
    (0.0, -330.0, -65.0),     # chin
    (-225.0, 170.0, -135.0),  # outer corner of the person's left eye
    (225.0, 170.0, -135.0),   # outer corner of the person's right eye
    (-150.0, -150.0, -125.0), # left mouth corner
    (150.0, -150.0, -125.0),  # right mouth corner
], dtype="double")

# Matching indices in the MediaPipe Face Mesh topology (468/478 points), in the SAME
# order as MODEL_POINTS above.
FACE_LANDMARK_IDX = [1, 152, 33, 263, 61, 291]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class WSClient:
    """Minimal RFC 6455 WebSocket client (stdlib only, text frames < 64 KiB)."""

    def __init__(self, host: str, port: int):
        self.sock = socket.create_connection((host, port), timeout=5)
        key = "k1-mimic-head-key=="
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

    def send_nowait(self, msg: dict) -> None:
        """Send without waiting for a reply. The robot's RPC was slow during testing
        (2026-08-12), so waiting for the reply inside the video loop froze the whole
        camera feed. Fire-and-forget: any reply that arrives later sits in the socket
        buffer and is discarded by the next send()/drain()/close()."""
        data = json.dumps(msg).encode()
        mask = os.urandom(4)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        n = len(data)
        if n < 126:
            hdr = struct.pack(">BB", 0x81, 0x80 | n)
        else:
            hdr = struct.pack(">BBH", 0x81, 0x80 | 126, n)
        try:
            self.sock.sendall(hdr + mask + masked)
        except OSError as e:
            print(f"[mimic-head] send_nowait failed: {e}")

    def drain(self) -> None:
        """Discard any reply left in the buffer (best-effort, non-blocking). Call
        periodically while using send_nowait, otherwise the buffer grows without
        bound in a long session."""
        try:
            while select.select([self.sock], [], [], 0)[0]:
                if not self.sock.recv(4096):
                    break
        except OSError:
            pass

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def main():
    host = os.environ.get("ROBOT_IP")
    if not host:
        print("Set ROBOT_IP (e.g. ROBOT_IP=<robot-ip> python3 mimicHead.py)")
        sys.exit(1)
    port = int(os.environ.get("ROBOT_MOTION_PORT", "8765"))

    print(f"[mimic-head] connecting to ws://{host}:{port} ...", flush=True)
    ws = WSClient(host, port)
    print("[mimic-head] connected. Press ESC in the webcam window to quit.", flush=True)

    mp_face = mp.solutions.face_mesh

    cap = cv2.VideoCapture(0)
    last_send = 0.0
    send_interval = 1.0 / SEND_HZ
    frame_w, frame_h = None, None
    camera_matrix, dist_coeffs = None, np.zeros((4, 1))
    prev_rvec, prev_tvec = None, None
    smooth_yaw_deg, smooth_pitch_deg = None, None

    try:
        with mp_face.FaceMesh(max_num_faces=1, refine_landmarks=True,
                              min_detection_confidence=0.6,
                              min_tracking_confidence=0.6) as face_mesh:
            while cap.isOpened():
                ok, frame = cap.read()
                if not ok:
                    continue

                if frame_w is None:
                    # Approximate pinhole camera: focal length ~ image width, optical
                    # center at the image center, no lens distortion.
                    frame_h, frame_w = frame.shape[:2]
                    focal = frame_w
                    center = (frame_w / 2, frame_h / 2)
                    camera_matrix = np.array([
                        [focal, 0, center[0]],
                        [0, focal, center[1]],
                        [0, 0, 1],
                    ], dtype="double")

                frame.flags.writeable = False
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = face_mesh.process(rgb)
                frame.flags.writeable = True

                now = time.monotonic()
                due = (now - last_send) >= send_interval

                if results.multi_face_landmarks:
                    face_lm = results.multi_face_landmarks[0].landmark
                    # Debug dots on the 6 landmarks in use (visual feedback that
                    # detection is picking the right points).
                    for idx in FACE_LANDMARK_IDX:
                        p = face_lm[idx]
                        cv2.circle(frame, (int(p.x * frame_w), int(p.y * frame_h)), 3, (0, 255, 0), -1)

                    if due:
                        image_points = np.array([
                            (face_lm[idx].x * frame_w, face_lm[idx].y * frame_h)
                            for idx in FACE_LANDMARK_IDX
                        ], dtype="double")

                        ok_pnp, rvec, tvec = cv2.solvePnP(
                            MODEL_POINTS, image_points, camera_matrix, dist_coeffs,
                            rvec=prev_rvec, tvec=prev_tvec,
                            useExtrinsicGuess=prev_rvec is not None,
                            flags=cv2.SOLVEPNP_ITERATIVE)

                        if ok_pnp:
                            last_send = now
                            prev_rvec, prev_tvec = rvec, tvec
                            rot_mat, _ = cv2.Rodrigues(rvec)
                            pose_mat = cv2.hconcat((rot_mat, np.zeros((3, 1))))
                            _, _, _, _, _, _, euler = cv2.decomposeProjectionMatrix(pose_mat)
                            pitch_deg, yaw_deg, _roll_deg = euler.flatten()
                            # decomposeProjectionMatrix can return the pitch "flipped"
                            # (>90 or <-90) because of solvePnP's sign ambiguity;
                            # normalize to -90..90 before converting.
                            if pitch_deg > 90:
                                pitch_deg -= 180
                            elif pitch_deg < -90:
                                pitch_deg += 180

                            # Jump limit + EMA: filters spikes that get past
                            # useExtrinsicGuess (landmark noise is still present).
                            if smooth_yaw_deg is None:
                                smooth_yaw_deg, smooth_pitch_deg = yaw_deg, pitch_deg
                            else:
                                yaw_deg = smooth_yaw_deg + clamp(
                                    yaw_deg - smooth_yaw_deg, -MAX_DELTA_DEG, MAX_DELTA_DEG)
                                pitch_deg = smooth_pitch_deg + clamp(
                                    pitch_deg - smooth_pitch_deg, -MAX_DELTA_DEG, MAX_DELTA_DEG)
                                smooth_yaw_deg = smooth_yaw_deg * (1 - SMOOTH_ALPHA) + yaw_deg * SMOOTH_ALPHA
                                smooth_pitch_deg = smooth_pitch_deg * (1 - SMOOTH_ALPHA) + pitch_deg * SMOOTH_ALPHA

                            yaw = clamp(YAW_SIGN * np.radians(smooth_yaw_deg) / 1.4, -1.0, 1.0) * YAW_MAX
                            pitch_norm = clamp(PITCH_SIGN * np.radians(smooth_pitch_deg) / 0.8, -1.0, 1.0)
                            pitch = PITCH_MIN + (pitch_norm + 1.0) / 2.0 * (PITCH_MAX - PITCH_MIN)

                            ws.send_nowait({"op": "rotate_head", "pitch": round(pitch, 3),
                                             "yaw": round(yaw, 3)})
                            print(f"[mimic-head] yaw_deg={smooth_yaw_deg:.1f} pitch_deg={smooth_pitch_deg:.1f} "
                                  f"-> yaw={yaw:.2f} pitch={pitch:.2f}", flush=True)

                ws.drain()
                cv2.imshow("Mimic Head (ESC to quit)", cv2.flip(frame, 1))
                if cv2.waitKey(1) & 0xFF == 27:
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[mimic-head] re-centering head before exit...")
        try:
            print(ws.send({"op": "rotate_head", "pitch": 0.0, "yaw": 0.0}))
        except Exception as e:  # noqa: BLE001
            print(f"[mimic-head] re-centering failed (ignoring): {e}")
        ws.close()
        print("[mimic-head] stopped.")


if __name__ == "__main__":
    main()
