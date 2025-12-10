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
        
        # After the initial AI move, we are waiting for the human player to move.
        self.scan_count = 1
        time.sleep(30)
        self.start_board_scan()
        self.game_state = "WAITING_FOR_PLAYER_MOVE"
        self.log_manual_action(initial_move)
        
    def log_manual_action(self, move):
        self.get_logger().info("==============================================")
        self.get_logger().info(f"ACTION REQUIRED: MANUALLY execute move: {move}")
        self.get_logger().info("Press ENTER in this terminal when the move is complete.")
        self.get_logger().info("==============================================")


    # ====================================================================
    # 2. ROBOT EXECUTION (COMMENTED OUT)
    # ====================================================================
    # def move_robot(self, move_uci: str): ... (commented out)
    # def robot_response_callback(self, future: Future): ... (commented out)


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
        if self.game_state == "WAITING_FOR_PLAYER_MOVE" and self.is_key_pressed():
            
            # 💡 Read and discard the input (e.g., the Enter key press)
            sys.stdin.readline() 

            # Trigger the second scan after the human has moved
            self.scan_count = 2
            self.get_logger().info("Terminal input detected. Beginning second scan (after human move).")
            self.game_state = "SCAN_TWO"
            self.start_board_scan()
            
        elif self.game_state == "WAITING_FOR_PLAYER_MOVE":
             # Print prompt so the user knows when to hit Enter
             print(f"\rWaiting for move confirmation... (Press Enter) -> ", end='', flush=True)


    # ====================================================================
    # 5. MOVE CALCULATION
    # ====================================================================
    def calculate_human_move(self):
        self.game_state = "CALCULATING_PLAYER_MOVE"

        prev_key = None
        for key in self.previous_board_state:
            if key not in self.current_board_state:
                self.get_logger().info(f'Piece left square: {key}')
                prev_key = key

        curr_key = None
        for key in self.current_board_state:
            if key not in self.previous_board_state:
                self.get_logger().info(f'Piece arrived at square: {key}')
                curr_key = key

        if prev_key and curr_key:
            self.player_move = prev_key + curr_key
        else:
            self.player_move = "INVALID_MOVE"
            self.get_logger().error("Could not calculate a simple player move. Using 'INVALID_MOVE'.")
        
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
        self.log_manual_action(ai_move)
        
        # After the AI move, reset scan count to 1 and wait for the human to make their move.
        self.scan_count = 1
        self.game_state = "WAITING_FOR_PLAYER_MOVE"


def main(args=None):
    rclpy.init(args=args)
    game_operation = GameOperation()
    rclpy.spin(game_operation)
    game_operation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()