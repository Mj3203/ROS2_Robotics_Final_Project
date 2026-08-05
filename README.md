# Project Overview

Created an autonomous chess-playing robot that uses a Kinova Gen3 Lite robot arm, RGB camera, and a custom compressed air end effector programmed by an Arduino Nano.

---
## How It Works

1. **Vision** — A camera captures the board. ArUco tags at the corners of the chess board allows the homography node warp the image into a top-down view, correcting for any angle/position.
2. **Object Detection** — A custom trained YOLO model scans the feed and maps every detected piece to a board square, producing a JSON representation of the current board state.
3. **Chess AI** — The scanned board state is compared against legal chess moves to determine what the human just played. Stockfish receives the validated move and calculates the best response.
4. **Pick and Place** — The robot arm uses MoveIt2 to plan and execute its moves.
5. **Game Operation** — A central node loops the steps listed above in a specified order, depending on which color the player chooses to play.
6. **Display Output** — Camera feeds, game status, and move history are shown on a physical display.

Each portion is independent, so each package/node can be debugged, restarted, or replaced independently of one another.

---

## Game Sequence

At startup, the player is asked which color they would like to play: **White** or **Black**.

### If the robot plays White (moves first)

1. The robot immediately asks the chess engine for its opening move.
2. The arm executes that move on the board.
3. The robot then waits for the human to respond.

### If the robot plays Black (waits for the human)

1. The robot waits for the human to make the first move.
2. The human moves a piece, then presses **ENTER** to signal "your turn."

### Each turn, from the robot's perspective

1. **Wait for the human.** The human makes their move physically and presses the button.
2. **Scan the board.** `game_operation_pkg` calls `ScanBoard`; `computer_vision_pkg` runs YOLO and returns the current piece layout.
3. **Validate the move.** `game_operation_pkg` calls `ValidateMove`; `chess_ai_pkg` compares the scan against the legal moves from the current position.
   - If the scan doesn't match any legal move, the move is rejected with a reason and the player must redo their move (back to step 1).
4. **Ask the AI for a reply.** `game_operation_pkg` calls `GetBestMove`. `chess_ai_pkg` applies the player's move internally, runs Stockfish, and returns the robot's best move and game status if necessary (e.g. check, checkmate, stalemate, draw).
5. **Check for game over.** If the status is checkmate/stalemate/draw, the loop ends and the result is reported.
6. **Execute the robot's move.** Otherwise `game_operation_pkg` calls `MoveRobot` with the move and the capture flag; `pick_and_place_pkg` drives the arm:
   - **Normal move:** hover over the source square → descend → suction on → lift → travel to the target square → descend → suction off → retract.
   - **Capture handling:** if the move is a capture, the arm first picks up the enemy piece on the target square and drops it into a side basket, **then** moves its own piece onto that square.
7. **Restart the loop.** Control returns to step 1 for the human's next move.

Throughout, `game_operation_pkg` publishes the current state to `game_status_feed`, so `display_output_pkg` always shows the latest move, status, and log.

---
