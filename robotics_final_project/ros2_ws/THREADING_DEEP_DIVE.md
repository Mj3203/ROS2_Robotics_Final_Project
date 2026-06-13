# Threading & Callbacks Deep Dive — The Second-Move Bug

This document explains, from scratch, every threading and callback problem we hit while building
the chess robot — and how each was diagnosed and fixed. It assumes **no prior knowledge** of
threads, ROS2, or executors. Each idea starts with one plain-English sentence, then a real-world
analogy, then the technical definition. Read it top to bottom; later sections lean on earlier ones.

The story in one line: *the robot's first move always worked, but the second move froze forever.*
It took four layers of investigation to find out why. Here's the whole journey.

---

## 1. Background Concepts

### What is a thread? (and `threading.Thread` vs. an executor thread)

**Plain English:** A thread is a single worker that runs one line of your program at a time; a
program can have several threads running different work at once.

**Analogy:** Think of a kitchen. One **cook** can only make one dish at a time, start to finish. If
you want two dishes cooked simultaneously, you need two cooks. A thread is a cook.

**Technical:** A thread is an independent sequence of execution within a process. Python lets you
create one with `threading.Thread(target=some_function)` — it runs `some_function` on a brand-new
cook while your other cooks keep working.

**The key distinction for this whole document — two *kinds* of threads:**

- A **normal thread** is one *you* created with `threading.Thread(...)`. It runs whatever function
  you handed it. ROS knows nothing about it. In our code, `motion_worker` runs on a normal thread:
  ```python
  self.worker_thread = threading.Thread(target=self.motion_worker, daemon=True)
  self.worker_thread.start()
  ```
  ([pick_and_place_node.py:47-48](src/pick_and_place_pkg/pick_and_place_pkg/pick_and_place_node.py#L47-L48))

- An **executor thread** is a thread that a **ROS2 executor** created and controls. Its only job is
  the executor's loop: wait for work, then run the right callback.

At the operating-system level **these are the same kind of thing** — there is no special "executor
thread" species. The difference is entirely about *who owns it and what loop it runs*. This sounds
like a technicality. It is actually the root cause of the final bug. Hold onto it.

### What is a ROS2 executor? What does "spinning" mean?

**Plain English:** An executor is the part of ROS that actually runs your callback functions;
"spinning" is it looping forever, looking for work to run.

**Analogy:** The executor is the **head chef** who watches the order tickets. Spinning is the chef
standing at the pass, continuously checking "any new tickets? cook them. any new tickets? cook
them," over and over, all night.

**Technical:** An executor owns one or more threads and runs a loop: *check which entities have
pending work → run their callbacks → repeat*. `rclpy.spin(node)` hands your node to a
single-threaded executor and runs that loop. A `MultiThreadedExecutor` runs the same loop but with
a **pool of worker threads**, so multiple callbacks can run at the same time. Our arm node uses:
```python
executor = MultiThreadedExecutor(num_threads=8)
executor.add_node(node)
threading.Thread(target=spin_executor, daemon=True).start()   # spin in the background
```
([pick_and_place_node.py:733-744](src/pick_and_place_pkg/pick_and_place_pkg/pick_and_place_node.py#L733-L744))

### What is a callback? How does it get called?

**Plain English:** A callback is a function you write but never call yourself — ROS calls it for you
when something happens.

**Analogy:** You leave a note with the receptionist: "*When a package arrives, open it and put it on
my desk.*" You don't stand at the door waiting. The receptionist (the executor) runs your
instructions when the trigger happens.

**Technical:** A callback is a function registered against an entity (a subscription, a timer, a
service). When that entity has work — a message arrives, a timer fires, a service request comes in —
the executor invokes the callback **on one of its threads**. You never call it directly. Example:
`handle_move_robot` is the callback for the `move_robot` service
([pick_and_place_node.py:698](src/pick_and_place_pkg/pick_and_place_pkg/pick_and_place_node.py#L698)).

**Crucial subtlety:** code only runs "on an executor thread" when the executor *dispatches it as a
callback*. If your code calls a function **directly** (e.g. `self.moveit2.plan(...)`), that function
runs on **whatever thread made the call** — not magically on an executor thread. This is the second
half of the root cause.

### What is a callback group? `MutuallyExclusive` vs. `Reentrant`

**Plain English:** A callback group is a label on your callbacks that tells the executor which ones
are allowed to run at the same time.

**Analogy:** Back to the kitchen. Some dishes share the single oven — only one can be in it at a
time. Other dishes can be made side by side with no conflict. A callback group is the rule that
says "these tickets share the oven (one at a time)" vs. "these can all go at once."

**Technical:** Every callback belongs to a group. Two kinds:

- **`MutuallyExclusiveCallbackGroup`** — only **one** callback from this group runs at any instant.
  The others wait their turn. (The shared oven.)
- **`ReentrantCallbackGroup`** — **any number** of callbacks from this group can run simultaneously,
  including the same callback overlapping itself. ("Reentrant" = safe to re-enter while already
  running.)

If you don't assign a group, callbacks land in the node's **default group, which is mutually
exclusive**. This surprises people: adding more threads does nothing on its own, because the default
group still forbids overlap. You need *both* multiple threads **and** a permissive group to get real
concurrency. In our node, MoveIt is given a reentrant group so its many internal callbacks can run
together:
```python
self.moveit_callback_group = ReentrantCallbackGroup()
self.moveit2 = MoveIt2(node=self, ..., callback_group=self.moveit_callback_group)
```
([pick_and_place_node.py:68-77](src/pick_and_place_pkg/pick_and_place_pkg/pick_and_place_node.py#L68-L77))

### What does "blocking" mean? What is a blocking call?

**Plain English:** A blocking call is a line of code that does not return until its work is finished,
so the thread running it can do nothing else in the meantime.

**Analogy:** You phone a friend and stay on hold. Until they pick up, you're stuck holding the
phone — you can't cook, clean, or answer the door. You are *blocked*.

**Technical:** A blocking call suspends its thread until the operation completes.
`self.moveit2.wait_until_executed()` blocks for the *entire* duration of an arm motion — up to ~14
seconds. While a thread is blocked inside a call, it cannot run anything else, including other
callbacks. *Who* is blocked (a normal thread vs. an executor thread) determines whether it's
harmless or catastrophic.

### What is a ROS2 service? How is request/response different from a topic?

**Plain English:** A service is a request-and-reply between two nodes: one asks a question and waits
for a single answer. A topic is a one-way broadcast with no reply.

**Analogy:** A **service** is a phone call — you call, ask, and wait for the answer before hanging
up. A **topic** is a radio station — the broadcaster just transmits; listeners tune in or not, and
nobody calls back.

**Technical:** A service has a `.srv` definition with a **Request** and a **Response**. The client
calls (often `call_async`) and receives exactly one matching response. A topic uses
publisher/subscriber: a publisher sends messages; any number of subscribers receive them; there is
no reply. In our system, `game_operation` *calls the `move_robot` service* to command a move, and
`pick_and_place` *publishes on the `move_complete` topic* to announce the move finished — a
deliberate mix of both patterns (see Section 4).

### What is a deadlock?

**Plain English:** A deadlock is when two parties are each waiting for the other to go first, so
neither ever moves.

**Analogy:** Two people meet in a narrow hallway. Each steps aside the same way, waits for the other
to pass, and neither does. They wait forever, politely frozen.

**Technical:** A deadlock occurs when thread A holds a resource that thread B needs while waiting for
a resource that B holds (or, in our case, the *only* worker is busy waiting on work that only that
same worker could perform). Nothing is broken; everything is just stuck waiting.

### What is a "wait set"? What does `rcl_wait` do? Why does the driving thread matter?

This is the most important concept in the document, and the least known. Read it slowly.

**Plain English:** The wait set is the list of all the things the executor is currently watching for
work. `rcl_wait` is the executor going to sleep until *something in that list* has work to do.

**Analogy:** The executor is a security guard watching a wall of monitors. The **wait set** is the
set of monitors plugged into the wall. The guard **`rcl_wait`s** — dozes off — and an alarm wakes
him the instant *any plugged-in monitor* shows motion. Here's the catch: **if a camera isn't
plugged into the wall, the guard never gets an alarm for it, no matter what happens in front of that
camera.** He'll sleep right through it.

**Technical:** Each loop, the executor gathers all of a node's entities (subscriptions, timers,
services, clients, action clients) into a **wait set** and passes it to the middleware via
`rcl_wait`. `rcl_wait` blocks the executor's spin thread until at least one entity in the set
becomes ready, *or* until someone calls `executor.wake()` (which pokes a special "guard condition"
to force the executor awake to rebuild the set). When an entity is **created or destroyed**, the
wait set must be rebuilt so the executor watches the correct things.

**Why the driving thread matters:** rclpy is **not safe** when entities are created/destroyed on one
thread while the executor is rebuilding or sleeping on the wait set on another thread. If MoveIt's
action machinery spins up and tears down internal entities **on a normal thread you spawned**, that
churn happens *outside* the executor's coordinated loop. The wake-up notification can be missed, and
the executor is left asleep on a **stale wait set** — a monitor wall that no longer has the
`move_robot` service plugged in. The service request arrives, but no alarm fires. That is a **lost
wakeup**, and it is exactly what froze move 2.

---

## 2. The Problems We Encountered (chronological)

There was never a *single* bug. There were **four layered problems**. Each fix revealed the next.

### Problem 1 — Executor deadlock

**What happened:** In the earliest design, `handle_move_robot` ran the MoveIt planning and motion
**directly inside the service callback**. That callback runs *on an executor thread*. But MoveIt's
own responses (the action results that tell it a motion finished) also need an executor thread to be
processed. The executor thread was busy sitting inside `handle_move_robot`, waiting for MoveIt —
while MoveIt was waiting for the executor. Classic deadlock (see the hallway analogy).

Conceptually the bad version looked like this — motion *inside* the callback:
```python
def handle_move_robot(self, request, response):
    src, dst = request.best_uci[:2], request.best_uci[2:]
    self.move_piece(src, dst)        # <-- runs MoveIt on the executor thread, blocks ~14s
    response.robot_status_message = "DONE"
    return response
```

**Why it worked in tune mode but failed in serve mode:** In `tune`/`test` modes, the motion is
launched from `main()` on the **main thread**, while the executor spins separately in the
background:
```python
threading.Thread(target=spin_executor, daemon=True).start()
...
node.task_tune()        # <-- motion driven from the MAIN thread, not a callback
```
([pick_and_place_node.py:744-750](src/pick_and_place_pkg/pick_and_place_pkg/pick_and_place_node.py#L744-L750))

Because the motion ran on the main thread, the executor's threads stayed **free** to process
MoveIt's responses — no deadlock. But in **serve** mode the motion ran *inside the service
callback*, i.e. on an executor thread, so the executor got stuck waiting on itself. Same motion
code, completely different outcome, purely because of **which thread drove it** — the theme of this
entire document.

### Problem 2 — Service callback occupied for 14 seconds

**What happened:** We broke the deadlock by giving MoveIt a `ReentrantCallbackGroup` (so its
responses could be processed on other threads even while the service callback ran). Move 1 now
completed. But the service callback was *still running the full 14-second motion*. From the
executor's point of view, the `move_robot` service callback was **occupied for 14 seconds straight**.

Because the service lived in a mutually-exclusive group, the executor would not dispatch a *second*
`move_robot` request until the first callback returned — and worse, a callback that hogs a slot for
that long after a full motion left the service unable to take new requests. Same bad shape as
Problem 1's code: `handle_move_robot` did not return until `move_piece()` had completely finished.

### Problem 3 — Worker thread, but `motion_result.wait()` still blocked the callback

**What happened:** Next we moved the motion onto a separate worker thread, intending to free the
callback. But the callback still *waited* for the worker to finish before returning:
```python
def handle_move_robot(self, request, response):
    self.motion_queue.put((src, dst, is_capture))
    motion_result.wait()             # <-- still blocks here for the full ~14s
    response.robot_status_message = "DONE"
    return response
```
The motion now ran on the worker thread, but `handle_move_robot` blocked on `motion_result.wait()`
for the entire move. **The callback was still occupied for 14 seconds** — identical symptom to
Problem 2, just a different line doing the blocking. This version was reverted.

### Problem 4 — Lost wakeup from a foreign thread driving rclpy (the real root cause)

**What happened:** We switched to the **async/accept pattern**: `handle_move_robot` enqueues the
move and returns `ACCEPTED` in about **1 millisecond**, and a daemon `motion_worker` thread runs the
motion and announces completion on a topic. The current code:
```python
def motion_worker(self):
    while rclpy.ok():
        src, dst, is_capture = self.motion_queue.get(timeout=1.0)   # waits for a move
        ...
        success = self.move_piece(src, dst)        # <-- drives MoveIt from a NORMAL thread
        self.move_complete_pub.publish(msg)        # <-- publishes from a NORMAL thread
```
([pick_and_place_node.py:667-695](src/pick_and_place_pkg/pick_and_place_pkg/pick_and_place_node.py#L667-L695))
```python
def handle_move_robot(self, request, response):
    ...
    self.motion_queue.put((src, dst, is_capture))
    response.robot_status_message = "ACCEPTED"
    return response                                # <-- returns in ~1 ms
```
([pick_and_place_node.py:698-712](src/pick_and_place_pkg/pick_and_place_pkg/pick_and_place_node.py#L698-L712))

This fixed Problems 2 and 3 — the callback no longer blocks. **Move 1 works cleanly.** But move 2
froze, every time, identically. And critically, `handle_move_robot` was now so fast that "the
callback is occupied" *could not* be the explanation.

The reason: `motion_worker` is a **normal thread** (Problem-1 theme returns). It drives *all* of
MoveIt's rclpy activity — sending action goals, which **creates and destroys internal entities**,
and publishing — from a thread the executor does not own. After motion 1 completed, that entity
churn on the foreign thread left the executor's **wait set stale**. The executor stayed asleep in
`rcl_wait` and **never woke up** to dispatch the second `move_robot` request. A textbook lost wakeup.

**The proof — the py-spy dump.** With the node frozen on move 2, we attached py-spy
(`py-spy dump --pid <pid> --native`) and got a snapshot of *every thread*. The important threads:

```
Thread 28244 (idle): "Thread-2 (motion_worker)"
    ... PyThread_acquire_lock_timed → wait → get (queue.py:180)
    motion_worker (pick_and_place_node.py:670)          <-- asleep on an EMPTY queue

Thread 28245 (idle): "Thread-3 (spin_executor)"
    ... ddsrt_cond_wait → rmw_wait → rcl_wait
    _wait_for_ready_callbacks (executors.py:778)
    spin (executors.py:359)                              <-- the executor, parked in rcl_wait

Thread 28246..28253 (idle): "ThreadPoolExecutor-0_0 ... _7"
    ... _worker (concurrent/futures/thread.py:89)        <-- all 8 callback workers idle
```

**What the dump proved:**

- Every thread is **idle**. The executor (`Thread-3`) is alive and parked in `rcl_wait`. The 8
  callback worker threads are all idle. `motion_worker` (`Thread-2`) is blocked in `queue.get()` —
  meaning **nothing was ever put on the queue**, so `handle_move_robot` was **never called**.
- The chain is unmistakable: the request reached the middleware, but `rcl_wait` never reported the
  service ready → the executor stayed asleep → the callback never ran → nothing was queued →
  `motion_worker` slept forever. A **lost wakeup**.

**What the dump ruled out:**

- ❌ **Deadlock** — no thread waits on a lock another holds; all are idle.
- ❌ **Thread starvation** — all 8 worker threads are idle, *not* busy. (This is why bumping the
  pool from 4 to 8 threads changed nothing.)
- ❌ **A blocking callback** — no callback is on any stack; `handle_move_robot` is not running.
- ❌ **A crashed executor** — `Thread-3` is alive and in `rcl_wait`, not gone. (Our crash-logging
  wrapper printed nothing, consistent with this.)

That left exactly one explanation standing: the executor is healthy but **deaf** — asleep on a
stale wait set after a foreign thread drove MoveIt. Problem 4 identified.

---

## 3. What We Tried — every fix, the theory behind it, and why it did or didn't work

1. **`ReentrantCallbackGroup` for MoveIt2.** *Theory:* let MoveIt's responses be processed on other
   threads while the service callback runs, breaking the deadlock. *Result:* **fixed Problem 1** —
   move 1 could now complete. *But:* the service callback was still occupied for 14 seconds
   (Problem 2 surfaced).

2. **Worker thread with `motion_queue` + `motion_result.wait()`.** *Theory:* move the motion off the
   executor so the callback is free. *Result:* the motion did move off the executor, but
   `handle_move_robot` still blocked on `motion_result.wait()` for ~14 s — **same symptom, different
   line.** Reverted.

3. **Async/accept pattern.** *Theory:* make `handle_move_robot` return `ACCEPTED` in ~1 ms and report
   completion later on the `move_complete` topic, so the callback is never occupied. *Result:*
   **fixed Problems 2 and 3** — move 1 works cleanly. *But:* it introduced **Problem 4** — the
   motion now runs on a foreign thread driving rclpy, causing the lost wakeup.

4. **`ReentrantCallbackGroup` on the service + 8 executor threads.** *Theory:* maybe the service was
   getting stuck "busy," or the pool was starved. *Result:* **did not fix Problem 4.** The py-spy
   dump showed all threads idle — the problem was never thread count or group assignment. It was
   *which thread owned the rclpy calls.* This is the moment the wait-set explanation became the only
   one that fit.

5. **`MultiThreadedExecutor` + `ReentrantCallbackGroup` on `game_operation_node`.** *Theory:* maybe
   the *sender* (`game_operation`, which runs a single-threaded `rclpy.spin()`) was failing to
   deliver request #2. *Result:* **reverted.** A direct `ros2 service call /move_robot ...` from a
   fresh shell *also* hung, and `game_operation` provably delivered its other calls fine on the same
   turn (scan, validate, AI all succeeded). That put the fault squarely on the `pick_and_place`
   side. (A **watchdog** was added to `game_operation` regardless, so a missed completion now flips
   the game to `ERROR` after 60 s instead of hanging forever — that's the
   `Watchdog: no completion from arm within 60s` line you saw, and it stays in as a safety net.)

---

## 4. The Resolution — the timer rewrite

The fix keeps everything that worked (the fast, non-blocking `handle_move_robot`) and removes the
one thing that didn't (the foreign thread driving MoveIt).

**The change, step by step:**

- **Delete** the raw `threading.Thread` worker:
  ```python
  # remove these:
  self.worker_thread = threading.Thread(target=self.motion_worker, daemon=True)
  self.worker_thread.start()
  ```
- **Replace** it with a ROS **timer** in its own `MutuallyExclusiveCallbackGroup`, firing every
  0.05 s:
  ```python
  self.motion_cb_group = MutuallyExclusiveCallbackGroup()
  self.motion_timer = self.create_timer(0.05, self.motion_timer_callback,
                                         callback_group=self.motion_cb_group)
  ```
- **Drain the queue from the timer callback** — the same move logic that used to live in
  `motion_worker`, now run from a callback the executor owns:
  ```python
  def motion_timer_callback(self):
      try:
          src, dst, is_capture = self.motion_queue.get_nowait()
      except queue.Empty:
          return                       # nothing to do this tick
      success = True
      if is_capture:
          success = self.capture_piece(dst)
      if success:
          if self.is_castling_move(src, dst):
              success = self.move_piece(src, dst)
              if success:
                  rook_src, rook_dst = self.get_castling_rook_move(src, dst)
                  success = self.move_piece(rook_src, rook_dst)
          else:
              success = self.move_piece(src, dst)
      msg = String(); msg.data = "SUCCESS" if success else "ERROR"
      self.move_complete_pub.publish(msg)
  ```
- **`handle_move_robot` is unchanged** — it stays lightweight: validate, `motion_queue.put(...)`,
  return `ACCEPTED` immediately
  ([pick_and_place_node.py:698-712](src/pick_and_place_pkg/pick_and_place_pkg/pick_and_place_node.py#L698-L712)).

**Why this works when nothing before did — connect it back to Section 1's wait set:**

1. **All rclpy work now runs on executor threads.** The timer is an executor-owned entity, so its
   callback (and therefore every MoveIt call inside it) is dispatched *by the executor, on the
   executor's own threads*. MoveIt's entity churn now happens **inside the executor's coordinated
   loop**, not on a foreign thread racing it. The wait set can no longer be corrupted by
   out-of-band entity creation/destruction. (Directly fixes Problem 4 / the lost wakeup.)

2. **The periodic timer makes "deaf forever" impossible.** Recall `rcl_wait` sleeps until an entity
   becomes ready. A timer that fires every 0.05 s *is* an entity that becomes ready 20 times a
   second — so `rcl_wait` is forced to **return on every tick**, and the executor **re-checks all of
   its entities, including the `move_robot` service.** Even if a wake-up notification were missed,
   the very next tick catches the pending request. The guard can no longer sleep through an alarm,
   because his own alarm clock rings constantly. (This is the security-guard wall, now with a timer
   that wakes him every 50 ms to re-scan every monitor.)

3. **Motions still can't overlap.** The timer sits in a `MutuallyExclusiveCallbackGroup`, so the
   executor will not start a second `motion_timer_callback` while one is still running its ~14-second
   move. No `motion_in_progress` flag needed — the group guarantees it. Meanwhile MoveIt's
   `ReentrantCallbackGroup` lets its action responses run on the *other* executor threads so the
   in-flight motion can actually complete, and the **reentrant `move_robot` service** can be
   dispatched on yet another free thread even mid-motion — so request #2 gets queued immediately
   instead of being lost.

The net effect: the callback stays fast (keeps the Problem-2/3 fix), the motion runs on
executor-owned threads (removes Problem 4), motions serialize safely, and the executor is
structurally incapable of going deaf.

---

## 5. Key Lessons

- **Never do rclpy work from a thread you spawned yourself** — *especially* action clients or
  anything that creates/destroys entities. Drive it from an executor-owned callback (a timer,
  subscription, or service handler) so it runs on executor threads and the executor stays aware of
  its own state. Almost every problem in this document traces back to violating this once.

- **Fixing a deadlock ≠ fixing a blocked service entity.** Problem 1 (deadlock) and Problem 2
  (callback occupied 14 s) are different failures with different fixes. Breaking the deadlock let
  move 1 run; it did nothing for the service being hogged afterward. Solve them as separate things.

- **Thread starvation ≠ a lost wakeup.** Starvation means all threads are *busy* (add threads to
  help). A lost wakeup means all threads are *idle* but the executor never gets told there's work
  (more threads do nothing). The py-spy dump distinguished them instantly: **all threads idle ⇒ not
  starvation ⇒ lost wakeup.** Diagnose which one you have before "fixing" it.

- **A py-spy dump is the right tool for a frozen ROS2 node.** `py-spy dump --pid <pid> --native`
  shows exactly where every thread is parked, with no code changes and no rebuild. It turned weeks
  of guessing into a five-minute certainty. Reach for it the moment a node hangs.

- **The async/accept pattern is correct for long-running ROS2 services** — return an `ACCEPTED`
  acknowledgement fast and report completion later (here, on the `move_complete` topic). But the
  long work it kicks off **must run on an executor-owned callback, not a foreign thread.** The
  pattern was right; the threading underneath it was wrong.

- **When to use which callback group:**
  - **`MutuallyExclusiveCallbackGroup`** — when callbacks must *not* overlap: serialized work (our
    motion timer — only one move at a time), or anything touching shared state without its own
    locking.
  - **`ReentrantCallbackGroup`** — when callbacks must be free to run concurrently: a handler that
    has to make progress *while another callback in the same group is still running* (MoveIt's
    action responses during a motion; a service that must stay dispatchable mid-motion).
  - And remember: the default (unspecified) group is **mutually exclusive**, so concurrency needs
    *both* a `MultiThreadedExecutor` and a permissive group — neither alone is enough.

---

*Companion docs: [SECOND_MOVE_FIX.md](SECOND_MOVE_FIX.md) (the test that localized the bug and the
staged fixes), [HANDOFF.md](HANDOFF.md) (full session history). The watchdog and the timer rewrite
are the durable outcomes of this investigation.*
