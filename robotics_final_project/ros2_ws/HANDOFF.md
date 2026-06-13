# ROS2 Chess Robot — Session Handoff
**Repo:** https://github.com/Mj3203/ROS2_Robotics_Final_Project.git
**Date:** June 13, 2026
**Hardware:** Kinova Gen3 Lite arm, Arduino Nano (CH341), suction-cup end effector + motor relay
**Stack:** ROS2 Jazzy, MoveIt2 2.10, pymoveit2, YOLO, Stockfish, Docker (Ubuntu 24.04)

---

## 1. Project Overview

An autonomous chess-playing robot that sees the board with a camera, decides moves with Stockfish, and physically moves the pieces with a Kinova Gen3 Lite arm and a suction-cup end effector. A camera feed is warped to a top-down view via ArUco markers, a YOLO model detects piece positions, `chess_ai_node` owns the authoritative `chess.Board()` and validates the human's move + picks the robot's reply, and `pick_and_place_node` executes the move on the arm. `game_operation_node` orchestrates the turn-by-turn loop and publishes status to a display.

---

## 2. What Happened This Session

- **AI move generation hardened:** `chess_ai_node` now filters out promotions for the AI (`get_restricted_legal_moves()` + `get_top_moves()`), so the robot never picks a promotion move. Stockfish search depth was lowered to **10** (`set_depth(10)`) for faster responses.
- **Castling support** confirmed in `pick_and_place_node` (`is_castling_move` / `get_castling_rook_move` → moves king then rook). Capture support uses `capture_piece(dst)` first, then the move.
- **Motor control added** to `move_piece` / `capture_piece` (`motor_control("motor on"/"motor off")` around the pick → release window). Arduino firmware was updated externally so both relays default OFF on power-up; **pyserial** was added to the image so serial commands work.
- **Board calibration** in `constants.py`: per-column square widths (d/e narrower), a `BOARD_ROTATION_RAD = math.radians(-1.6)` rotation correction applied in `square_center_in_world()`, and a flat `+0.0035 m` x-offset for columns d–h.
- **Test tasks** added: `test_capture`, `test_castling`, `test_move`, plus an `origin` tune mode.
- **Major debugging effort** on why the **second** AI move never executes during a live game (see §4). Root-cause work led to an **async accept/report refactor** of the `move_robot` service (see §5). Move 1 now works end to end; move 2 is still blocked.
- Two earlier fix attempts were tried and **reverted**: a pick_and_place "worker thread that the service callback waits on" (still blocked the callback), and a game_operation `MultiThreadedExecutor` + `ReentrantCallbackGroup` change (reverted before the async fix existed).

---

## 3. Build/Run Gotcha Learned This Session

**Python edits do NOT take effect on a node restart alone — you must `colcon build` first.** Despite `--symlink-install` in the Dockerfile, live edits to `src/.../*.py` are not reflected at runtime in this container (stale install / `__pycache__`). Always:
```bash
cd /robotics_final_project/ros2_ws
colcon build --packages-select <pkg>
source install/setup.bash      # in each shell
# then restart the affected node(s)
```

---

## 4. Current Blocking Issue

**The first AI move completes perfectly; the second AI move never executes.**

Last live run (robot = White):
- **Move 1 (e2e4):** works end to end.
  - `game_operation`: `AI move received: e2e4` → `Move accepted by arm` (**+13 ms**, instant) → `AI move complete` (**+14 s**).
  - `pick_and_place`: `[SERVICE] Received move request: e2e4` → `[SERVICE] Move queued — returning ACCEPTED` → motion logs → `[WORKER] Move complete — published status: SUCCESS`.
- **Human move e7e5:** validated correctly by `chess_ai_node`.
- **Move 2 (g1f3):** **stuck.**
  - `game_operation`: logs `AI move received: g1f3 (capture=False).` and then **goes silent** — no `Move accepted by arm`.
  - `pick_and_place`: shows **nothing** after move 1's `[WORKER] Move complete` — i.e. **`[SERVICE] Received move request: g1f3` never appears.**

So the second `move_robot` request never reaches `pick_and_place`'s `handle_move_robot`, and `game_operation` never gets an ACK back, so it sits in `SENDING_TO_ARM` forever.

**Key finding that overturned the earlier theory:** with the async fix, move 1's service callback now returns in ~5 ms (no longer blocks 14 s), yet **move 2 still fails identically.** So the cause is **not** "a long-blocking service callback wedges the service." The fault is either:
- (a) `pick_and_place`'s `move_robot` service stops dispatching a second request for some other reason (executor / callback-group / MoveIt-from-worker-thread interaction), **or**
- (b) `game_operation` (single-threaded `rclpy.spin`, chaining `call_async` from deep inside nested done-callbacks) fails to deliver request #2.

**The discriminating test (not yet run) — safe, no arm motion:** after move 1 completes, from a sourced shell:
```bash
ros2 service call /move_robot custom_interface/srv/MoveRobot "{best_uci: 'xx', is_capture: false}"
```
`xx` is rejected by the length check before anything is queued, so it returns instantly with no motion.
- Returns `ERROR: Invalid UCI` → **service is alive** → bug is in **game_operation delivery** → re-apply the `MultiThreadedExecutor` + `ReentrantCallbackGroup` change to game_operation (it was reverted before the async fix existed; the two together are the likely fix).
- Hangs with no response → **service is wedged** → deeper `pick_and_place` executor/MoveIt issue.

> Note: `self.future` reuse in `game_operation` was investigated and ruled out — rclpy keeps each future alive in the client's `_pending_requests`, so reassigning `self.future` does not drop callbacks.

---

## 5. Architecture of the Async Fix

Designed to keep the `move_robot` service callback short so it can't block the executor.

**`pick_and_place_node` (`handle_move_robot`):**
- Validates the UCI, then **`motion_queue.put((src, dst, is_capture))`** and **returns `"ACCEPTED"` immediately** (no motion in the callback).
- A dedicated daemon **`motion_worker`** thread (started in `__init__`) pulls from the queue and runs the move: `capture_piece(dst)` if capture, then king+rook if `is_castling_move`, else `move_piece(src, dst)`.
- On completion the worker **publishes the result on the `move_complete` topic** (`std_msgs/String`, payload `"SUCCESS"` or `"ERROR"`).
- Callback groups (kept from the deadlock fix): service in a `MutuallyExclusiveCallbackGroup`, MoveIt2 in a `ReentrantCallbackGroup`, `MultiThreadedExecutor(num_threads=4)`.

**`game_operation_node`:**
- `call_pick_and_place` sends the `MoveRobot` request as before.
- `pick_and_place_callback` now treats the `ACCEPTED` response as "arm executing" → sets `game_state = "ARM_MOVING"` (only if still `SENDING_TO_ARM`). It no longer advances the game.
- A new subscription **`move_complete`** → **`move_complete_callback`** is what advances the game: on `SUCCESS` → `WAITING_FOR_PLAYER_MOVE`; on `ERROR` → `ERROR`. It ignores messages unless `game_state` is `SENDING_TO_ARM`/`ARM_MOVING` (guards a fast-completion ordering race).
- Still runs on a single-threaded `rclpy.spin()` (the MultiThreadedExecutor attempt was reverted).

**No `.srv` change** — the existing `MoveRobot.robot_status_message` carries `"ACCEPTED"`. No `custom_interface` rebuild needed.

So: game_operation sends the move → gets `ACCEPTED` instantly → waits for a `move_complete` topic message to proceed.

---

## 6. Current State of Files

The bulk of prior work (node refactors, package.xml/setup.py cleanup, README/SETUP_DOCKER, calibration) is **committed**. The current **uncommitted** delta is the async fix + chess_ai depth.

| File | Status | Notes |
|---|---|---|
| `pick_and_place_pkg/pick_and_place_node.py` | ⚠ Modified (uncommitted) | Async `handle_move_robot` (enqueue + `ACCEPTED`), `motion_worker` thread, `move_complete` publisher, `[PLAN]/[EXEC]/[WAIT]/[SERVICE]` debug logging, motor control, castling/capture, `test_*` + `origin` tune modes |
| `game_operation_pkg/game_operation_node.py` | ⚠ Modified (uncommitted) | `move_complete` subscription + `move_complete_callback`; `pick_and_place_callback` → `ARM_MOVING` on ACCEPTED; single-threaded `rclpy.spin` |
| `chess_ai_pkg/chess_ai_node.py` | ⚠ Modified (uncommitted) | No-promotion AI filter; `set_depth(10)` |
| `pick_and_place_pkg/constants.py` | ✅ Committed | Square-width generation, `BOARD_ROTATION_RAD=-1.6`, d–h `+0.0035` offset, `BOARD_Z`, `BOARD_ORIGIN` |
| `display_output_pkg/display_output_node.py` | ✅ Committed | Already shows `ARM_MOVING` ("Arm moving piece") — no change needed for the async fix |
| `README.md`, `SETUP_DOCKER.md` | ✅ Committed | Project README + Jazzy two-image Docker doc |
| `Dockerfile.dependencies` | ✅ Committed | `pyserial` added to pip install |

> The `move_complete` topic + `ARM_MOVING` flow are uncommitted and only live in `pick_and_place_node.py` / `game_operation_node.py`. Rebuild + restart both for them to take effect.

---

## 7. Environment and Run Commands

**Build containers (if not already built):**
```bash
docker build -f Dockerfile.kortex_jazzy -t kortex_jazzy .
docker build -f Dockerfile.dependencies -t chessbot .
```

**Terminal 1 — start container and launch the full arm stack (driver + MoveIt + RViz in one command):**
```bash
docker run -it --privileged --net=host \
  -v /dev:/dev \
  -v ~/workspaces/ROS2_Robotics_Final_Project/robotics_final_project/ros2_ws/src:/robotics_final_project/ros2_ws/src \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix chessbot

ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=192.168.1.10
```

**Additional terminals — open a shell in the running container (then source the workspace):**
```bash
docker exec -it $(docker ps -q) bash --login
source /robotics_final_project/ros2_ws/install/setup.bash
```

**After any Python edit (REQUIRED — see §3):**
```bash
cd /robotics_final_project/ros2_ws && colcon build --packages-select <pkg> && source install/setup.bash
```

**Run the nodes (one per shell):**
```bash
ros2 run computer_vision_pkg raw_camera_feed
ros2 run computer_vision_pkg homography_transform
ros2 run object_detection_pkg scan_and_detect
ros2 run chess_ai_pkg chess_ai
ros2 run display_output_pkg display_output
ros2 run pick_and_place_pkg pick_and_place        # serve mode (waits for move_robot)
ros2 run game_operation_pkg game_operation        # prompts robot W/B, drives the game
```

**Tune / test commands (pick_and_place):**
```bash
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=home
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=center
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=origin
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=square -p tune_square:=e4
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=lift -p tune_square:=e4
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=direct -p tune_square:=h8
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=hold
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=tune -p tune_mode:=open

# Motion test tasks (place real pieces on the relevant squares first):
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=test_move -p src:=e2 -p dst:=e4
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=test_capture -p test_square:=e4
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=test_castling -p castling_side:=kingside
ros2 run pick_and_place_pkg pick_and_place --ros-args -p task:=test_castling -p castling_side:=queenside

ros2 run pick_and_place_pkg snake_test
```

---

## 8. Next Steps (Priority Order)

1. **Run the discriminating test (§4):** `ros2 service call /move_robot … "{best_uci: 'xx', …}"` after move 1. This is the single most important next action — it tells you whether to fix `pick_and_place` (service wedged) or `game_operation` (delivery).
2. **If service is alive → fix `game_operation` delivery:** re-apply `MultiThreadedExecutor(num_threads≥4)` + a `ReentrantCallbackGroup` on the clients/timer (and ideally the new `move_complete` subscription). It was reverted earlier, but only ever tested against the *blocking* pick_and_place; with the async fix it's the likely cure.
3. **If service is wedged → investigate `pick_and_place`:** suspect the worker thread calling pymoveit2 from a non-executor thread, or thread/callback-group exhaustion after a full motion. Consider whether `move_group` action callbacks are starving the `MutuallyExclusiveCallbackGroup` service.
4. **Add a guard / timeout** so a missed `move_complete` doesn't hang the game forever (e.g. a watchdog in `game_operation` that flags `ERROR` if no completion within N seconds).
5. **Once the move loop is solid:** verify capture (`f3e5`-style) and castling end-to-end in a live game (capture path runs `capture_piece` for the first time in a real game — not yet exercised live).
6. **Verify the vision pipeline** (homography → YOLO scan → display) with the real board under the current Jazzy container.
7. **Re-confirm calibration** (`BOARD_ROTATION_RAD`, d–h offset, `BOARD_Z`) on the physical board; `h7`/`h8` reachability was historically marginal.
8. **Clean up:** remove the `[PLAN]/[EXEC]/[WAIT]` debug logging from `pick_and_place_node` once the bug is fixed; commit the async fix.

---

## Key Log Signatures (for the next person)

- Healthy move: `[SERVICE] Move queued — returning ACCEPTED` → motion → `[WORKER] Move complete — published status: SUCCESS` (pick_and_place); `Move accepted by arm` → `AI move complete` (game_operation).
- The bug: `game_operation` logs `AI move received: <uci>` then silence; `pick_and_place` logs **no** `[SERVICE] Received move request: <uci>` for the second move.
