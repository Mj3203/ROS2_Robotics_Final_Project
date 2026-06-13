# Second-Move Bug — Test & Staged Fixes

The first AI move works; the second never reaches `pick_and_place`. This doc has:
1. The discriminating test (decides which fix to apply).
2. **Fix A** — apply if the test returns `ERROR` (bug is in `game_operation` delivery).
3. **Fix B** — apply if the test **hangs** (bug is in `pick_and_place`, service wedged).

A **watchdog** is already applied to `game_operation` (active now, no action needed): if the arm
doesn't report completion within `ARM_TIMEOUT_S` (60s), the game flips to `ERROR` instead of
hanging forever. This is independent of A/B and stays in regardless.

---

## 1. The discriminating test

Run it **while the game is frozen on move 2** — i.e. after `game_operation` logs
`AI move received: <uci>` and goes silent. In a **separate** sourced terminal:

```bash
docker exec -it $(docker ps -q) bash --login
source /robotics_final_project/ros2_ws/install/setup.bash
ros2 service call /move_robot custom_interface/srv/MoveRobot "{best_uci: 'xx', is_capture: false}"
```

`xx` fails the length check before anything is queued → **no arm motion**, totally safe.

| Result | Meaning | Apply |
|---|---|---|
| `ERROR: Invalid UCI` instantly | service is alive; `game_operation` isn't delivering request #2 | **Fix A** |
| Hangs, no response | `pick_and_place` service is wedged | **Fix B** |

While frozen, also capture these for confirmation (cheap, read-only):
```bash
ros2 service list | grep move_robot          # is the service still advertised?
ros2 node info /pick_and_place_node           # is move_robot still listed under Service Servers?
```

---

## 2. Fix A — `game_operation` delivery (run multithreaded)

Hypothesis: the single-threaded executor can't deliver request #2 because it's issued from deep
inside a nested done-callback chain. Give it a thread pool + a reentrant group so a second thread
can dispatch the call while the first is still inside the chain.

**File:** `src/game_operation_pkg/game_operation_pkg/game_operation_node.py`

### A.1 — imports (top of file, near the other `from rclpy...` lines)
```python
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
```

### A.2 — in `__init__`, put every client/sub/timer in one reentrant group
Replace the current client/subscription/timer creation block with:
```python
        self.cb_group = ReentrantCallbackGroup()
        self.piece_detection_client = self.create_client(ScanBoard, 'scan_board', callback_group=self.cb_group)
        self.chess_ai_client = self.create_client(GetBestMove, 'get_best_move', callback_group=self.cb_group)
        self.validate_move_client = self.create_client(ValidateMove, 'validate_move', callback_group=self.cb_group)
        self.pick_and_place_client = self.create_client(MoveRobot, 'move_robot', callback_group=self.cb_group)
        self.move_complete_sub = self.create_subscription(
            String, 'move_complete', self.move_complete_callback, 10, callback_group=self.cb_group)
        self.game_status_pub = self.create_publisher(String, 'game_status_feed', 10)
        self.timer = self.create_timer(0.1, self.input_check_timer_callback, callback_group=self.cb_group)
```

### A.3 — swap `rclpy.spin()` for a MultiThreadedExecutor in `main()`
```python
def main(args=None):
    rclpy.init(args=args)
    game_operation = GameOperation()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(game_operation)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    game_operation.destroy_node()
    rclpy.shutdown()
```

### A.4 — race-safety note (important once multithreaded)
With multiple threads, two callbacks can touch `self.game_state` at the same instant. The existing
guards (`if self.game_state == "SENDING_TO_ARM"`, and the `move_complete_callback` state check)
cover the known orderings, but if you see flaky state transitions, wrap the read-modify-write of
`game_state` in a `threading.Lock`:
```python
import threading
# in setup_game:
self.state_lock = threading.Lock()
# then guard transitions, e.g.:
with self.state_lock:
    if self.game_state == "SENDING_TO_ARM":
        self.game_state = "ARM_MOVING"
```
Start without the lock; add it only if you observe a race.

### Build & run
```bash
cd /robotics_final_project/ros2_ws
colcon build --packages-select game_operation_pkg
source install/setup.bash
# restart game_operation
```

---

## 3. Fix B — `pick_and_place` service wedged

Hypothesis: after one full MoveIt motion, the executor can't dispatch the second service callback
(thread starvation, or the worker thread driving pymoveit2 leaves an executor thread blocked).
This one is more investigative than A — start cheap, escalate if needed.

**File:** `src/pick_and_place_pkg/pick_and_place_pkg/pick_and_place_node.py`

### B.1 — cheapest first attempt: more executor threads
In `main()`, bump the pool so MoveIt's reentrant callbacks can't starve the service:
```python
    executor = MultiThreadedExecutor(num_threads=8)
```
If the second move now goes through, that confirms thread starvation was the cause.

### B.2 — if B.1 doesn't fix it: confirm where it's stuck (while frozen)
```bash
# Is the service still advertised, or did the server drop it?
ros2 service list | grep move_robot
ros2 node info /pick_and_place_node

# Does the worker thread ever publish? (should be silent when wedged)
ros2 topic echo /move_complete
```
- Service **gone** → the node/server itself died or the callback group is permanently busy.
- Service **present** but call still hangs → executor isn't dispatching → threads are blocked.

### B.3 — structural candidate (bigger change, only if B.1/B.2 point to a blocked thread)
The motion runs on the hand-rolled `motion_worker` thread, calling pymoveit2
(`plan`/`execute`/`wait_until_executed`) which depend on executor threads dispatching MoveIt's
action callbacks. If `wait_until_executed()` is leaving an executor thread parked, options are:
- Give MoveIt2 its **own dedicated executor/thread** separate from the service executor, so the
  service callback group can never be starved by motion callbacks; **or**
- Add explicit logging right at the top of `handle_move_robot` *before* the length check to prove
  whether the callback is entered at all on request #2 (it already logs `[SERVICE] Received move
  request` — if that line never prints for the direct `ros2 service call`, the callback is genuinely
  not being dispatched, which points at the executor, not the code inside the callback).

### Build & run
```bash
cd /robotics_final_project/ros2_ws
colcon build --packages-select pick_and_place_pkg
source install/setup.bash
# restart pick_and_place
```

---

## 4. After it works
- Remove the `[PLAN]/[EXEC]/[WAIT]` debug logging from `pick_and_place_node`.
- Keep the watchdog.
- Commit the async fix + whichever of A/B fixed it.
- Verify capture and castling end-to-end in a live game (capture path not yet exercised live).
