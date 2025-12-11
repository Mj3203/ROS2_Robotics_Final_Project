# --- Create ROS2 Node --- #
import rclpy
from rclpy.node import Node
# --- ROS2 Message Types --- #
from std_msgs.msg import String
import os

# --- Custom Service Types --- #
from custom_interface.srv import GetBestMove  # Import the service type

from ament_index_python.packages import get_package_share_directory

# --- Chess Packages --- #
from stockfish import Stockfish
import chess

class ChessAIServer(Node):
    def __init__(self):
        super().__init__('chess_ai_server')
        self.get_logger().info('Chess AI Server node has started')
        
        # --- Chess Setup --- #
        stockfish_binary_path = "/robotics_final_project/ros2_ws/install/chess_ai_pkg/share/chess_ai_pkg/stockfish_engine/stockfish-ubuntu-x86-64-avx2"
        
        self.get_logger().info(f"Loading Stockfish from: {stockfish_binary_path}")

        try:
            # 2. Try initializing the engine
            self.stockfish = Stockfish(path=stockfish_binary_path) 
            self.stockfish.set_depth(23) 
            self.get_logger().info("Stockfish engine initialized successfully.")
            
        except Exception as e:
            # 3. If it fails, log the specific error
            self.get_logger().error(f"Failed to initialize Stockfish: {e}")
            # Do NOT exit, let the rest of __init__ run, but self.stockfish will not exist
        #Preserves the game state
        self.board = chess.Board() 
        
        # --- Service Server --- #
        self.srv = self.create_service(GetBestMove, 'get_best_move', self.get_best_move_callback)
        self.get_logger().info('ChessAIServer node is ready. Service server listening')
    
    def evaluate_board_state(self):
        """
        Checks for check, checkmate, stalemate, or draw after a move.
        Returns a status string or None.
        """

        # Checkmate
        if self.board.is_checkmate():
            self.get_logger().info("CHECKMATE detected!")
            outcome = self.board.outcome()
            result = outcome.result()  # "1-0", "0-1", or "1/2-1/2"
            self.get_logger().info(f"Game result: {result}")

            if result == "1-0":
                return "CHECKMATE_WHITE_WINS"
            elif result == "0-1":
                return "CHECKMATE_BLACK_WINS"
            else:
                return "CHECKMATE"

        # Stalemate
        if self.board.is_stalemate():
            self.get_logger().info("Stalemate detected!")
            return "STALEMATE"

        # Insufficient material
        if self.board.is_insufficient_material():
            self.get_logger().info("Draw — insufficient material.")
            return "DRAW_INSUFFICIENT_MATERIAL"

        # Repetition
        if self.board.can_claim_threefold_repetition():
            self.get_logger().info("Draw — threefold repetition possible.")
            return "DRAW_REPETITION"

        # Check only
        if self.board.is_check():
            self.get_logger().info("CHECK!")

        return None
    def get_mate_in_n(self):
        """
        Reads Stockfish evaluation and logs mate-in-N if found.
        Returns 'MATE_IN_#' or None.
        """
        eval_info = self.stockfish.get_evaluation()

        if eval_info["type"] == "mate":
            mate_value = eval_info["value"]  # positive = white mating, negative = black mating
            self.get_logger().info(f"Stockfish reports mate in {mate_value}")
            return f"MATE_IN_{mate_value}"

        return None

    def get_best_move_callback(self, request, response):
        players_move = request.players_move

        # --------------------------------------------------------
        # GAME START — AI moves first (white)
        # --------------------------------------------------------
        if players_move == "START_GAME":
            self.board = chess.Board()
            self.stockfish.set_fen_position(self.board.fen())

            ai_move = self.stockfish.get_best_move()
            self.board.push_uci(ai_move)

            self.get_logger().info(f"Initial Robot Move (White): {ai_move}")
            self.get_logger().info(f"FEN: {self.board.fen()}")

            # Checkmate detection after AI opening move
            state = self.evaluate_board_state()
            if state:
                response.best_move = state
                return response

            # Mate in N prediction
            mate_prediction = self.get_mate_in_n()
            if mate_prediction:
                self.get_logger().info(f"Mate prediction: {mate_prediction}")

            response.best_move = ai_move
            return response

        # --------------------------------------------------------
        # PLAYER MOVE RECEIVED
        # --------------------------------------------------------
        self.get_logger().info(f"Received player move: {players_move}")
        self.board.push_uci(players_move)

        # Detect check/checkmate/stalemate after player's move
        state = self.evaluate_board_state()
        if state:
            response.best_move = state
            return response

        # --------------------------------------------------------
        # AI MOVE CALCULATION
        # --------------------------------------------------------
        self.stockfish.set_fen_position(self.board.fen())
        ai_move = self.stockfish.get_best_move()
        self.board.push_uci(ai_move)

        self.get_logger().info(f"AI Move: {ai_move}")
        self.get_logger().info(f"New FEN: {self.board.fen()}")

        # Detect win/draw after AI move
        state = self.evaluate_board_state()
        if state:
            response.best_move = state
            return response

        # Mate-in-N prediction
        mate_prediction = self.get_mate_in_n()
        if mate_prediction:
            self.get_logger().info(f"Mate prediction: {mate_prediction}")

        response.best_move = ai_move
        return response

def main(args=None):
    rclpy.init(args=args)
    chess_ai_server = ChessAIServer()
    rclpy.spin(chess_ai_server)
    chess_ai_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()