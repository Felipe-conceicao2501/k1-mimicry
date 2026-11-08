# Plan: adapting head/arm mimicry from the K1 to the T2

Research dated 2026-08-27, done **without physical access to a T2**, based on public Booster specs (booster.tech, docs.booster.tech, github.com/BoosterRobotics). Items marked **[CONFIRM]** can only be closed with access to a real robot or developer docs. Nothing here has been tested on a T2.

## Hardware differences that affect the code

| | K1 (current) | T2 |
|---|---|---|
| Total DoF | 22 | 31 |
| DoF per arm | 4 (shoulder pitch/roll + elbow pitch/yaw), **no wrist** | 7, most likely including wrist roll/pitch/yaw |
| Waist | none | 3 DoF (new) |
| Head | 2 DoF | 2 DoF (same) |
| Hand / end effector | optional in the SDK, **not installed** on this robot | configurable end effector, may include a right hand/gripper |
| Wrist camera | no | optional |
| Compute | n/a | Jetson Thor T5000 (Professional) |

## What has to change in each piece

### 1. `mimicHead.py`
Low risk. The logic (Face Mesh + solvePnP + `rotate_head` over WebSocket) does not depend on arm or waist DoF. It only needs confirmation that the T2 (or whatever stack runs on it) exposes a capability equivalent to `rotate_head`. **[CONFIRM]**

### 2. `mimicArms.py`
A larger change, not just a port. Today the code **pins the wrist to 0 on purpose** because the K1 has no such joint. On the T2 the wrist is a real joint, so this needs:
- Wrist orientation from video. MediaPipe Pose alone does not provide this reliably; it would need MediaPipe Hands, using palm orientation relative to the forearm.
- Three new angles in the WebSocket protocol (`move_arm` gains wrist roll/pitch/yaw, or a new op).
- Re-calibrated signs and limits. Like `YAW_SIGN`/`PITCH_SIGN` in `mimicHead.py`, these must be rediscovered live on the T2, since each robot has its own sign convention (as seen on the K1).

### 3. Hand / fingers (new capability)
If the T2 ships with a hand installed, mimicry could be extended to fingers. That would need MediaPipe Hands (21 points per hand) instead of Pose only, and the T2's finger-control API. **[CONFIRM]** — no public RPC IDs found.

### 4. The robot-side motion service (not in this repository)
This is the biggest piece, larger than the laptop scripts. It wraps the Booster C++ SDK (`B1LocoClient`) in a simple WebSocket protocol. For the T2:
- Confirm whether the T2's `booster_robotics_sdk` still uses `B1LocoClient` (as on the K1/T1) or a new class. The official public repos (`booster_robotics_sdk`, `booster_assets`) explicitly document only the **T1 and K1** at the time of writing. **[CONFIRM]**
- If the class is compatible, porting the adapter layer is mostly mechanical (config and joint names).
- If not, the adapter layer must be rewritten for the new SDK.
- Kinematics change (7-DoF arm vs. 4-DoF): any K1-specific angle/limit logic in the arm controller has to be redone, not just reused.

## Suggested order once a T2 is available

1. Run a `rotate_head` equivalent on the bare T2 (no extra software) to confirm basic SDK communication, as was done on the K1 originally.
2. Port the minimal motion service (arm without wrist first), replicating what already works on the K1, to prove the stack survives a robot change.
3. Only then extend to the wrist (3 new DoF) and, if applicable, fingers.
4. Keep the waist (3 DoF) out of the initial scope; there is no reason yet to mimic the waist from a webcam.

## Real blockers (cannot be resolved without the robot)

- Exact names of the new joints, angle limits and RPC IDs: only with the T2 SDK/developer docs in hand, or by asking Booster support directly.
- Sign convention of each new axis: only by testing live.
