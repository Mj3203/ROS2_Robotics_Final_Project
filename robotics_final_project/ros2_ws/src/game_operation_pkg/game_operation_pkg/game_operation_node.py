# --- Create ROS2 Node --- #
import rclpy
from rclpy.node import Node
from rclpy.task import Future
import time
import select # 💡 New import for checking terminal input
import sys # 💡 New import for stdin/stdout
 
# --- ROS2 Message Types --- #
from std_msgs.msg import String
 
# --- Custom Service Types --- #
from custom_interface.srv import GetBoardState, GetBestMove, MoveRobot
 
import json
 
class GameOperation(Node):
    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('game_operation_node')
        self.get_logger().info('Game Operation Node has started.')
 
        # --- Service Clients --- #
        self.piece_detection_client = self.create_client(GetBoardState, 'get_board_state')
        self.chess_ai_client = self.create_client(GetBestMove, 'get_best_move')
        # self.pick_and_place_client = self.create_client(MoveRobot, 'move_robot') # Commented out
 
        # --- Initializing --- #
        self.wait_for_all_services()
        self.previous_board_state = {}
        self.current_board_state = {}
        self.scan_count = 0
 
        # --- Timer for Input Check --- #
        # We replace the button subscriber with a timer that checks the keyboard periodically
        self.timer = self.create_timer(0.1, self.input_check_timer_callback)
 
        # --- Begin Loop --- #
        self.get_logger().info('Game Operation Node is ready.')
        self.start_initial_move_sequence()
 
    def wait_for_all_services(self):
        self.get_logger().info("Waiting for all services to connect")
        self.piece_detection_client.wait_for_service()
        self.chess_ai_client.wait_for_service()
        self.get_logger().info("All required services connected (Vision, AI)")
 
    # ====================================================================
    # 1. INITIAL MOVE
    # ====================================================================
    def start_initial_move_sequence(self):
        self.get_logger().info("Starting initial move sequence (White's move)")
        self.game_state = "CALLING_INITIAL_MOVE"
        
        request = GetBestMove.Request(players_move="START_GAME")
        self.future = self.chess_ai_client.call_async(request)
        self.future.add_done_callback(self.initial_move_callback)
    
    def initial_move_callback(self, future: Future):
        self.game_state = "INITIAL_MOVE_RECIEVED"
        try:
            response = future.result()
            initial_move = response.best_move
        except Exception as e:
            self.get_logger().error(f'Initial move service call failed: {e}')
            self.game_state = "ERROR"
            return
            
        self.get_logger().info(f"Received AI initial move: {initial_move}. Executing.")
 
        self.move_robot_with_human_assistance(initial_move)
 
    def move_robot_with_human_assistance(self, move_uci:str):
        self.get_logger().info(f"Commanding robot (manual human execution required currently) to execute move: {move_uci}")  
        
        self.log_manual_action(move_uci)
         
        self.game_state = "WAITING_FOR_HUMAN_TO_EXECUTE_AI_MOVE"
    
    def log_manual_action(self, move):
        self.get_logger().info("==============================================")
        self.get_logger().info(f"ACTION REQUIRED: MANUALLY execute move: {move}")
        self.get_logger().info("Press ENTER in this terminal when the move is complete.")
        self.get_logger().info("==============================================")

 
    # ====================================================================
    # 2. ROBOT EXECUTION (COMMENTED OUT)
    # ====================================================================
    #def move_robot(self, move_uci: str):
    #    """Send a move command to the robot."""
    #    self.get_logger().info(f"Commanding robot to execute move: {move_uci}")
    #    self.game_state = "CALLING_ROBOT_SERVICE"
    
    #    request = MoveRobot.Request(best_uci=move_uci)
    #    self.future = self.pick_and_place_client.call_async(request)
    #    self.future.add_done_callback(self.robot_response_callback)
    
    #def robot_response_callback(self, future: Future):
    #    """Handle robot status after executing move."""
    #    self.game_state = "ROBOT_SERVICE_COMPLETE"
    
    #    try:
    #        response = future.result()
    #        status_message = response.robot_status_message
    
    #        # Check for robot error
    #        if "ERROR" in status_message.upper():
    #            self.get_logger().error(f"Robot execution failed! Status: {status_message}")
    #            self.game_state = "ERROR"
    #            return
    #    except Exception as e:
    #        self.get_logger().error(f'Robot service call failed: {e}')
    #        self.game_state = "ERROR"
    #        return
    
    #    self.get_logger().info("Robot move complete. Begin first scan.")
    
        # Prepare for two-step scan
    #    self.scan_count = 1
    #    self.game_state = "SCAN_ONE"
    #    self.start_board_scan()
 
    # ====================================================================
    # 3. BOARD SCANNING
    # ====================================================================
    def start_board_scan(self):
        request = GetBoardState.Request(request_message="Requesting Board State")
        self.future = self.piece_detection_client.call_async(request)
        self.future.add_done_callback(self.board_scan_callback)
 
    def board_scan_callback(self, future: Future):
        try:
            response = future.result()
            board_json_string = response.board_json
            board_state = json.loads(board_json_string)
        except Exception as e:
            self.get_logger().error(f'Piece Detection service call failed: {e}')
            self.game_state = "ERROR"
            return
 
        if self.scan_count == 1:
            self.previous_board_state = board_state
            self.get_logger().info("Scan one completed (Before Player Move). Waiting for human player.")
            self.game_state = "WAITING_FOR_PLAYER_MOVE"
 
        elif self.scan_count == 2:
            self.current_board_state = board_state
            self.get_logger().info("Scan two completed (After Player Move). Calculating player move.")
            self.calculate_human_move()
 
    # ====================================================================
    # 4. TERMINAL INPUT TRIGGER (Replaces Button Callback)
    # ====================================================================
    def is_key_pressed(self):
        # 💡 Check if standard input (stdin) has data ready to be read (key press)
        # Timeout 0.0 means non-blocking check
        return select.select([sys.stdin], [], [], 0.0) == ([sys.stdin], [], [])
 
    def input_check_timer_callback(self):
        if self.is_key_pressed():    
            # 💡 Read and discard the input (e.g., the Enter key press)
            sys.stdin.readline()
 
            # -----------------------------------------
            # CASE 1: Human executed the AI's move → Scan 1
            # -----------------------------------------
            if self.game_state == "WAITING_FOR_HUMAN_TO_EXECUTE_AI_MOVE":
                self.get_logger().info("ENTER detected → Human completed AI move. Starting Scan 1.")
                self.scan_count = 1
                self.game_state = "SCAN_ONE"
                self.start_board_scan()
                return
 
            # -----------------------------------------
            # CASE 2: Human made THEIR move → Scan 2
            # -----------------------------------------
            if self.game_state == "WAITING_FOR_PLAYER_MOVE":
                self.get_logger().info("ENTER detected → Human completed their move. Starting Scan 2.")
                self.scan_count = 2
                self.game_state = "SCAN_TWO"
                self.start_board_scan()
                return
 
        # ===========================
        # PROMPTS (shown when waiting)
        # ===========================
        if self.game_state == "WAITING_FOR_HUMAN_TO_EXECUTE_AI_MOVE":
            print("\rExecute AI move and press ENTER… ", end="", flush=True)
 
        elif self.game_state == "WAITING_FOR_PLAYER_MOVE":
            print("\rHuman move complete? Press ENTER… ", end="", flush=True)
 
 
    # ====================================================================
    # 5. MOVE CALCULATION
    # ====================================================================

    def calculate_human_move(self):
        self.game_state = "CALCULATING_PLAYER_MOVE"

        prev = None   # square the black piece moved FROM
        curr = None   # square the black piece moved TO

        prev_board = self.previous_board_state
        curr_board = self.current_board_state

        # ------------------------------------------------------------
        # 1) Detect FROM-square (piece disappeared)
        # ------------------------------------------------------------
        prev_missing = []   # track multiple missing squares (castling)
        for sq in prev_board:
            if sq not in curr_board:
                prev_missing.append(sq)
                if prev is None:
                    prev = sq
                self.get_logger().info(f"Piece left square: {sq}")

        # ------------------------------------------------------------
        # 2) Detect TO-square (piece appeared)
        # ------------------------------------------------------------
        curr_added = []   # track multiple added squares (castling)
        for sq in curr_board:
            if sq not in prev_board:
                curr_added.append(sq)
                if curr is None:
                    curr = sq
                self.get_logger().info(f"Piece arrived at square: {sq}")

        # ------------------------------------------------------------
        # 3) CAPTURE CASE — piece changed but square existed before
        # ------------------------------------------------------------
        if curr is None: 
            for sq in curr_board:
                if sq in prev_board:
                    if curr_board[sq] != prev_board[sq]:   # piece changed → capture
                        curr = sq
                        self.get_logger().info(f"Piece arrived at (capture): {sq}")
                        break

        # ------------------------------------------------------------
        # SPECIAL CASE HANDLING FOR BLACK ONLY
        # ------------------------------------------------------------

        # ----------------------------------------------
        # BLACK CASTLING — kingside (e8→g8, h8→f8)
        # ----------------------------------------------
        if set(prev_missing) == {"e8", "h8"} and set(curr_added) == {"g8", "f8"}:
            self.player_move = "e8g8"
            self.get_logger().info("Detected BLACK CASTLING (king-side)")
            return self.call_chess_ai()

        # ----------------------------------------------
        # BLACK CASTLING — queenside (e8→c8, a8→d8)
        # ----------------------------------------------
        if set(prev_missing) == {"e8", "a8"} and set(curr_added) == {"c8", "d8"}:
            self.player_move = "e8c8"
            self.get_logger().info("Detected BLACK CASTLING (queen-side)")
            return self.call_chess_ai()

        # ----------------------------------------------
        # BLACK EN PASSANT
        # ----------------------------------------------
        if prev and curr:
            moved_piece = prev_board.get(prev)

            if moved_piece and "Pawn" in moved_piece:

                file_from = prev[0]
                rank_from = int(prev[1])
                file_to   = curr[0]
                rank_to   = int(curr[1])

                # Pawn moved diagonally
                if abs(ord(file_from) - ord(file_to)) == 1:

                    # If arrival square was previously empty → possible EP
                    prev_piece_on_arrival = prev_board.get(curr, None)
                    if prev_piece_on_arrival is None or "Pawn" not in prev_piece_on_arrival:

                        # Square where white pawn would be captured
                        ep_square = file_to + str(rank_from)

                        # If that square disappeared → EP confirmed
                        if ep_square in prev_missing:
                            self.player_move = prev + curr
                            self.get_logger().info("Detected BLACK EN PASSANT")
                            return self.call_chess_ai()

        # ----------------------------------------------
        # BLACK PROMOTION (pawn reaches rank 1)
        # ----------------------------------------------
        if prev and curr:
            moved_piece = prev_board.get(prev)
            if moved_piece and "Pawn" in moved_piece:
                if curr.endswith("1"):
                    self.player_move = prev + curr + "q"   # always promote to queen
                    self.get_logger().info("Detected BLACK PROMOTION")
                    return self.call_chess_ai()

        # ------------------------------------------------------------
        # DEFAULT MOVE (normal or capture)
        # ------------------------------------------------------------
        if prev and curr:
            self.player_move = prev + curr
        else:
            self.player_move = "INVALID_MOVE"
            self.get_logger().error("Could not calculate a simple black move. Using 'INVALID_MOVE'.")

        # ------------------------------------------------------------
        # Call AI response
        # ------------------------------------------------------------
        self.get_logger().info(f"Calculated Player Move: {self.player_move}. Calling AI.")
        self.call_chess_ai()


 
 
    # ====================================================================
    # 6. AI RESPONSE
    # ====================================================================
    def call_chess_ai(self):
        self.game_state = "CALLING_AI_SERVICE"
 
        request = GetBestMove.Request()
        request.players_move = self.player_move
        
        self.future = self.chess_ai_client.call_async(request)
        self.future.add_done_callback(self.ai_response_callback)
 
    def ai_response_callback(self, future: Future):
        try:
            response = future.result()
            ai_move = response.best_move
        except Exception as e:
            self.get_logger().error(f'AI service call failed: {e}')
            self.game_state = "ERROR"
            return
            
        self.get_logger().info("--------------------------------------------")
        self.get_logger().info(f"AI Move Found: {ai_move}.")
        
        # 💡 MANUAL TESTING LOGIC ADDED:
        self.move_robot_with_human_assistance(ai_move)
 
 
def main(args=None):
    rclpy.init(args=args)
    game_operation = GameOperation()
    rclpy.spin(game_operation)
    game_operation.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()