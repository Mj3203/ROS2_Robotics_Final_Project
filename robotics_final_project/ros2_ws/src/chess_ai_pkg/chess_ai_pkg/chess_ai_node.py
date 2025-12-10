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
            self.stockfish.set_depth(18) 
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

    def get_best_move_callback(self, request, response):
        #Extracts the last played move
        players_move = request.players_move

        if players_move == "START_GAME":
            self.board = chess.Board()
            #Sets the starting position
            self.stockfish.set_fen_position(self.board.fen())
            #Calculates the first move
            chess_ai_move = self.stockfish.get_best_move()
            #Updates the internal game logic
            self.board.push_uci(chess_ai_move)

            self.get_logger().info(f"Initial Robot Move (White): {chess_ai_move}")
            self.get_logger().info(f"Current FEN after initial move: {self.board.fen()}")

            #Send to client
            response.best_move = chess_ai_move
            return response
        
        else:
            self.get_logger().info(f'Received players move: {players_move}')
            #Updates the internal game logic with players move
            #If this move is illegal, this will crash the node
            self.board.push_uci(players_move)
            #Update Stockfish position and calculate the new move
            self.stockfish.set_fen_position(self.board.fen())
            chess_ai_move = self.stockfish.get_best_move()
            #Updates the internal game logic
            self.board.push_uci(chess_ai_move)

            self.get_logger().info(f'Calculated and applied AI move: {chess_ai_move}')
            self.get_logger().info(f'New FEN: {self.board.fen()}')

            #Send to client
            response.best_move = chess_ai_move
            return response

def main(args=None):
    rclpy.init(args=args)
    chess_ai_server = ChessAIServer()
    rclpy.spin(chess_ai_server)
    chess_ai_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()