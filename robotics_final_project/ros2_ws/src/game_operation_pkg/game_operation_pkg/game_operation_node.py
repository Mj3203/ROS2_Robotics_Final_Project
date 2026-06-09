import rclpy
import json
import select
import sys
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String
from custom_interface.srv import ScanBoard, GetBestMove, ValidateMove, MoveRobot


class GameOperation(Node):

    def __init__(self):
        super().__init__('game_operation_node')
        self.piece_detection_client = self.create_client(ScanBoard, 'scan_board')
        self.chess_ai_client = self.create_client(GetBestMove, 'get_best_move')
        self.validate_move_client = self.create_client(ValidateMove, 'validate_move')
        self.pick_and_place_client = self.create_client(MoveRobot, 'move_robot')
        self.game_status_pub = self.create_publisher(String, 'game_status_feed', 10)
        self.timer = self.create_timer(0.1, self.input_check_timer_callback)
        self.setup_game()
        self.wait_for_all_services()
        self.get_logger().info('Game Operation Node is ready.')
        self.start_game()

    def setup_game(self):
        self.game_state = "STARTING_SYSTEM"
        self.current_board_state = {}
        self.human_move = ""
        self.ai_move = ""
        self.ai_is_capture = False
        self.move_status = ""
        self.invalid_reason = ""
        self.game_status = ""
        self.move_number = 0
        robot_color = input("Is the robot WHITE or BLACK? (W/B): ").strip().upper()
        self.robot_color = "WHITE" if robot_color == "W" else "BLACK"
        self.human_color = "Black" if self.robot_color == "WHITE" else "White"
        self.ai_color = "White" if self.robot_color == "WHITE" else "Black"

    def wait_for_all_services(self):
        self.get_logger().info("Waiting for all services to connect")
        self.piece_detection_client.wait_for_service()
        self.chess_ai_client.wait_for_service()
        self.validate_move_client.wait_for_service()
        self.pick_and_place_client.wait_for_service()
        self.get_logger().info("All services connected.")

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            "game_state": self.game_state,
            "human_move": self.human_move,
            "ai_move": self.ai_move,
            "move_status": self.move_status,
            "invalid_reason": self.invalid_reason,
            "game_status": self.game_status,
            "move_number": self.move_number,
            "human_color": self.human_color,
            "ai_color": self.ai_color,
        })
        self.game_status_pub.publish(msg)

    def start_game(self):
        if self.robot_color == "WHITE":
            self.get_logger().info("Robot is WHITE. Requesting first move from AI.")
            self.call_chess_ai("START_GAME")
        else:
            self.get_logger().info("Robot is BLACK. Waiting for human to move first.")
            self.game_state = "WAITING_FOR_PLAYER_MOVE"
            self.get_logger().info("Make your move, then press ENTER.")
            self.publish_status()

    def is_key_pressed(self):
        return select.select([sys.stdin], [], [], 0.0) == ([sys.stdin], [], [])

    def input_check_timer_callback(self):
        if self.is_key_pressed():
            sys.stdin.readline()
            if self.game_state == "WAITING_FOR_PLAYER_MOVE":
                self.get_logger().info("ENTER detected -> Scanning board for human move.")
                self.game_state = "SCANNING"
                self.publish_status()
                self.scan_board()
                return

        if self.game_state == "WAITING_FOR_PLAYER_MOVE":
            print("\rMake your move, then press ENTER... ", end="", flush=True)

    def scan_board(self):
        self.game_state = "SCANNING"
        self.publish_status()
        request = ScanBoard.Request(request_message="Requesting Board Scan")
        self.future = self.piece_detection_client.call_async(request)
        self.future.add_done_callback(self.board_scan_callback)

    def board_scan_callback(self, future: Future):
        try:
            response = future.result()
            self.current_board_state = json.loads(response.board_json)
            self.get_logger().info("Board scan complete.")
        except Exception as e:
            self.get_logger().error(f'Board scan failed: {e}')
            self.game_state = "ERROR"
            self.publish_status()
            return
        self.call_validate_move()

    def call_validate_move(self):
        self.game_state = "VALIDATING_MOVE"
        self.publish_status()
        request = ValidateMove.Request()
        request.board_json = json.dumps(self.current_board_state)
        self.future = self.validate_move_client.call_async(request)
        self.future.add_done_callback(self.validate_move_callback)

    def validate_move_callback(self, future: Future):
        try:
            response = future.result()
            validated_move = response.validated_move
            self.invalid_reason = response.invalid_reason
        except Exception as e:
            self.get_logger().error(f'Validate move service call failed: {e}')
            self.game_state = "ERROR"
            self.publish_status()
            return

        self.human_move = validated_move

        if validated_move == "INVALID":
            self.get_logger().error(f"Invalid move detected: {self.invalid_reason}")
            self.get_logger().error("Please redo your move and press ENTER.")
            self.game_state = "WAITING_FOR_PLAYER_MOVE"
            self.move_status = "INVALID"
            self.publish_status()
            return

        self.invalid_reason = ""
        self.move_status = "VALID"
        self.get_logger().info(f"Valid human move: {validated_move}. Requesting AI response.")
        self.publish_status()
        self.call_chess_ai(validated_move)

    def call_chess_ai(self, players_move):
        self.game_state = "CALLING_AI"
        self.publish_status()
        request = GetBestMove.Request()
        request.players_move = players_move
        self.future = self.chess_ai_client.call_async(request)
        self.future.add_done_callback(self.ai_response_callback)

    def ai_response_callback(self, future: Future):
        try:
            response = future.result()
            ai_move = response.best_move
            self.game_status = response.game_status
            self.ai_is_capture = response.is_capture
        except Exception as e:
            self.get_logger().error(f'AI service call failed: {e}')
            self.game_state = "ERROR"
            self.publish_status()
            return

        self.game_state = "AI_COMPLETE"
        self.ai_move = ai_move
        self.publish_status()

        if self.game_status in ("CHECKMATE_WHITE_WINS", "CHECKMATE_BLACK_WINS", "CHECKMATE", "STALEMATE", "DRAW_INSUFFICIENT_MATERIAL", "DRAW_REPETITION"):
            self.get_logger().info(f"Game over: {self.game_status}")
            self.game_state = "GAME_OVER"
            self.publish_status()
            return

        self.move_number += 1
        self.get_logger().info(f"AI move received: {ai_move} (capture={self.ai_is_capture}).")
        self.call_pick_and_place(ai_move, self.ai_is_capture)

    def call_pick_and_place(self, move_uci: str, is_capture: bool):
        self.game_state = "SENDING_TO_ARM"
        self.publish_status()
        request = MoveRobot.Request()
        request.best_uci = move_uci
        request.is_capture = is_capture
        self.future = self.pick_and_place_client.call_async(request)
        self.future.add_done_callback(self.pick_and_place_callback)

    def pick_and_place_callback(self, future: Future):
        try:
            response = future.result()
            status = response.robot_status_message
        except Exception as e:
            self.get_logger().error(f'Pick and place service call failed: {e}')
            self.game_state = "ERROR"
            self.publish_status()
            return

        if "ERROR" in status.upper():
            self.get_logger().error(f"Robot execution failed: {status}")
            self.game_state = "ERROR"
            self.publish_status()
            return

        self.game_state = "WAITING_FOR_PLAYER_MOVE"
        self.get_logger().info("AI move complete. Make your move, then press ENTER.")
        self.publish_status()

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    game_operation = GameOperation()
    try:
        rclpy.spin(game_operation)
    except KeyboardInterrupt:
        pass
    game_operation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()