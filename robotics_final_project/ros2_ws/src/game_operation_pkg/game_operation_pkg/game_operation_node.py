# --- Create ROS2 Node --- #
import rclpy
from rclpy.node import Node
from rclpy.task import Future
import time

# --- ROS2 Message Types --- #
from sensor_msgs.msg import Image
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
        self.pick_and_place_client = self.create_client(MoveRobot, 'move_robot')

        # --- Topic Subscriber --- #
        self.button_subscriber = self.create_subscription(String, 'button_feed', self.button_callback, 10)

        # --- Initializing --- #
        self.wait_for_all_services()
        self.previous_board_state = {}
        self.current_board_state = {} 
        self.scan_count = 0

        # --- Begin Loop --- #
        self.get_logger().info('Game Operation Node is ready.')
        self.start_initial_move_sequence()

    def wait_for_all_services(self):
        self.get_logger().info("Waiting for all services to connect")

        self.piece_detection_client.wait_for_service()
        self.chess_ai_client.wait_for_service()
        self.pick_and_place_client.wait_for_service()

        self.get_logger().info("All services connected")

    # ====================================================================
    # 1. INITIAL MOVE
    # ====================================================================
    def start_initial_move_sequence(self):
        self.get_logger().info("Starting initial move")
        self.game_state = "CALLING_INITIAL_MOVE"
        
        #Keyword "START_GAME" to trigger the initial move on the server
        request = GetBestMove.Request(player_move="START_GAME") 
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
            
        self.get_logger().info(f"Received initial move: {initial_move}. Executing.")
        
        #Send to robot
        self.move_robot(initial_move)

    # ====================================================================
    # 2. ROBOT EXECUTION
    # ====================================================================
    def move_robot(self, move_uci: str):
        self.get_logger().info(f"Commanding robot to execute move: {move_uci}")
        self.game_state = "CALLING_ROBOT_SERVICE"
        
        request = MoveRobot.Request(best_uci=move_uci)
        self.future = self.pick_and_place_client.call_async(request)
        self.future.add_done_callback(self.robot_response_callback)

    def robot_response_callback(self, future: Future):
        self.game_state = "ROBOT_SERVICE_COMPLETE"
        try:
            response = future.result()
            status_message = response.robot_status_message
            if "ERROR" in status_message.upper():
                self.get_logger().error(f"Robot execution failed! Status: {status_message}")
                self.game_state = "ERROR"
                return
        except Exception as e:
            self.get_logger().error(f'Robot service call failed: {e}')
            self.game_state = "ERROR"
            return
            
        self.get_logger().info("Robot move complete. Begin first scan")
        
        #Begin first scan
        self.scan_count = 1
        self.game_state = "SCAN_ONE"
        self.start_board_scan()

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
            self.get_logger().info("Scan one completed. Waiting for human player")
            self.game_state = "WAITING_FOR_PLAYER_MOVE"

        elif self.scan_count == 2:
            self.current_board_state = board_state
            self.get_logger().info("Scan two completed. Calculating player move.")
            self.calculate_human_move()

    # ====================================================================
    # 4. BUTTON TRIGGER (Manages the two-step scan for human move)
    # ====================================================================
    def button_callback(self, msg: String):
        if self.game_state == "WAITING_FOR_PLAYER_MOVE":
            self.scan_count = 2
            self.get_logger().info("Button pressed. Beginning second scan.")
            self.game_state = "SCAN_TWO"
            self.start_board_scan()
        else:
            # IGNORE button presses that happen during Robot motion, AI thinking, or setup.
            self.get_logger().warn(f"Ignoring button signal. Current state: {self.game_state}")

    # ====================================================================
    # 5. MOVE CALCULATION
    # ====================================================================
    def calculate_human_move(self):
        self.game_state = "CALCULATING_PLAYER_MOVE"
        
        prev_key = None
        for key in self.previous_board_state:
            if key not in self.current_board_state:
                self.get_logger().info(f'{key} is not present in the current board state')
                prev_key = key

        curr_key = None
        for key in self.current_board_state:
            if key not in self.previous_board_state:
                self.get_logger().info(f'{key} is not present in the previous board state')
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

        request = GetBestMove.Request(players_move=self.player_move)
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
            
        self.get_logger().info(f"AI returned move: {ai_move}.")
        
        #Transition to robot execution
        self.move_robot(ai_move)

def main(args=None):
    rclpy.init(args=args)
    game_operation = GameOperation()
    rclpy.spin(game_operation)
    game_operation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()