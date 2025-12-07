import rclpy
from rclpy.node import Node
from custom_interface.srv import GetBoardState 
from sensor_msgs.msg import Image # ROS2 message types
from std_msgs.msg import String # Topic type for the button signal
import json

class Orchestrator(Node):
    def __init__(self):
        super().__init__('orchestrator_node') 

        # --- State Variables ---
        self.previous_board_state = {}
        self.current_board_state = {}
        self.move_state = "WAITING_FOR_AI_MOVE" 

        # 1. Subscriptions
        # Topic from Chess AI (Robot's next move)
        self.ai_sub = self.create_subscription(String, 'ai_move_command', self.ai_move_callback, 10)
        # Topic from Button Node (Signals when player move is complete/stable)
        self.button_sub = self.create_subscription(String, 'move_button_signal', self.button_callback, 10)

        # 2. Publishers (For sending move to Robot and receiving move from Robot)
        # Placeholder: Assuming you will publish to the Robot Node for execution
        self.robot_pub = self.create_publisher(String, 'robot_execute_move', 10) 
        # Placeholder: Assuming the Robot Node publishes confirmation when move is done
        self.robot_done_sub = self.create_subscription(String, 'robot_move_done', self.robot_done_callback, 10) 
        # Publisher to send human's detected move back to Chess AI
        self.ai_move_pub = self.create_publisher(String, 'human_move_result', 10)

        # 3. Service Client (Connects to PieceDetection)
        self.client = self.create_client(GetBoardState, 'get_board_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Piece Detection Service not available, waiting...')
        
        self.get_logger().info(f'Orchestrator node started. State: {self.move_state}')


    # ====================================================================
    # 1. State: WAITING_FOR_AI_MOVE
    # ====================================================================
    def ai_move_callback(self, msg):
        """Triggered when the Chess AI calculates and sends the robot's move."""
        if self.move_state == "WAITING_FOR_AI_MOVE":
            ai_move = msg.data
            self.get_logger().info(f'AI Move received: {ai_move}. Sending to robot.')
            
            # Send the move command to the robot node
            self.robot_pub.publish(msg) 
            self.move_state = "EXECUTING_ROBOT_MOVE"
        else:
            self.get_logger().warn(f'Ignoring AI move in state: {self.move_state}')
            
    # ====================================================================
    # 2. State: EXECUTING_ROBOT_MOVE
    # ====================================================================
    def robot_done_callback(self, msg):
        """Triggered when the Robot Node confirms the execution is complete."""
        if self.move_state == "EXECUTING_ROBOT_MOVE":
            self.get_logger().info('Robot confirmed move completion. Awaiting human move.')
            self.move_state = "WAITING_FOR_HUMAN_MOVE_SIGNAL"
        else:
            self.get_logger().warn(f'Ignoring robot done signal in state: {self.move_state}')


    # ====================================================================
    # 3. States: WAITING_FOR_HUMAN_MOVE_SIGNAL / WAITING_FOR_HUMAN_MOVE_CONFIRMATION
    # ====================================================================
    def button_callback(self, msg):
        """Triggered when the player signals the board is ready."""
        if self.move_state == "WAITING_FOR_HUMAN_MOVE_SIGNAL":
            self.get_logger().info('Button pressed (1st time). Capturing "before" state.')
            self.send_request()
            self.move_state = "PROCESSING_HUMAN_MOVE_1"
            
        elif self.move_state == "WAITING_FOR_HUMAN_MOVE_CONFIRMATION":
            self.get_logger().info('Button pressed (2nd time). Capturing "after" state.')
            self.send_request()
            self.move_state = "PROCESSING_HUMAN_MOVE_2"
            
        else:
            self.get_logger().warn(f'Ignoring button press in state: {self.move_state}')


    # ====================================================================
    # Service Call & Response Handler
    # ====================================================================
    def send_request(self):
        """Helper function to send the service request asynchronously."""
        request = GetBoardState.Request()
        request.request_message = "Capture board state."
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """Handles the asynchronous response from the PieceDetection service."""
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.move_state = "ERROR_RETRY" # Fallback to a retry state if needed
            return

        board_dict = json.loads(response.board_json)
        
        if self.move_state == "PROCESSING_HUMAN_MOVE_1":
            self.previous_board_state = board_dict
            self.get_logger().info('✅ Captured BEFORE state. Awaiting button confirmation.')
            self.move_state = "WAITING_FOR_HUMAN_MOVE_CONFIRMATION"
            
        elif self.move_state == "PROCESSING_HUMAN_MOVE_2":
            self.current_board_state = board_dict
            self.move_state = "ANALYZING_AND_SENDING"
            self.analyze_and_send_move()
            
            
    # ====================================================================
    # 7. State: ANALYZING_AND_SENDING
    # ====================================================================
    def analyze_and_send_move(self):
        """Detects the move made by the human player and sends it back to the AI."""
        
        from_sq, to_sq, piece = self.detect_move(self.previous_board_state, self.current_board_state)
        
        if from_sq and to_sq:
            human_move = from_sq + to_sq # e.g., "e7e5"
            self.get_logger().info(f'*** ♟️ HUMAN MOVE DETECTED: {piece} moved {human_move} ***')
            
            # Publish the move back to the Chess AI
            msg = String()
            msg.data = human_move
            self.ai_move_pub.publish(msg)
            
        else:
            self.get_logger().error('Could not reliably detect a single human move. Sending error/empty string.')
            # You might send an error signal or request a board reset here
            
        # Reset the loop
        self.move_state = "WAITING_FOR_AI_MOVE"
        self.get_logger().info('Loop reset. Waiting for new move from AI.')


    def detect_move(self, before, after):
        """
        Compares two board dictionaries to detect a single chess move.
        Returns a tuple (from_square, to_square, piece_name) or (None, None, None).
        (Your specific logic remains unchanged)
        """
        prev_key = None
        disappeared_pieces = 0
        
        # 1. Find the square where a piece disappeared
        for square, piece in before.items():
            if square not in after or after[square] != piece:
                prev_key = square 
                disappeared_pieces += 1
                piece_name = piece

        # 2. Find the square where a piece appeared/changed
        curr_key = None
        appeared_pieces = 0
        for square, piece in after.items():
            if square not in before or before[square] != piece:
                curr_key = square 
                appeared_pieces += 1

        # 3. Check for a clean single move (one piece gone, one piece arrived)
        if disappeared_pieces == 1 and appeared_pieces == 1:
            return (prev_key, curr_key, piece_name)
        
        if disappeared_pieces == 2 and appeared_pieces == 2:
             self.get_logger().warn('Castling or complex move detected. Requires advanced logic.')
             
        return (None, None, None)


def main(args=None):
    rclpy.init(args=args)
    orchestrator_node = Orchestrator()

    try:
        rclpy.spin(orchestrator_node)
    except KeyboardInterrupt:
        pass
    
    orchestrator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()