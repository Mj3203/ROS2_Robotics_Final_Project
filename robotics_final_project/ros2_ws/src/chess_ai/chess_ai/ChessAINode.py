import rclpy
from rclpy.node import Node
from game_ai.srv import GetAIMove

import chess
import chess.engine

class ChessAINode(Node):
    def __init__(self):
        super().__init__("chess_ai")

        # Load stockfish engine
        self.engine = chess.engine.SimpleEngine.popen_uci("/usr/bin/stockfish")

        # keep track of game state
        self.board = chess.Board()

        # service server
        self.srv = self.create_service(GetAIMove, "get_ai_move", self.handle_get_ai_move)

        self.get_logger().info("Chess AI node ready.")

    def handle_get_ai_move(self, request, response):
        # update board with the human's move
        human_move = chess.Move.from_uci(request.human_move)
        self.board.push(human_move)

        # engine computes response
        result = self.engine.play(self.board, chess.engine.Limit(time=0.1))
        ai_move = result.move.uci()

        # apply AI move to internal board
        self.board.push(result.move)

        response.ai_move = ai_move
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ChessAINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()