import rclpy
import json
from rclpy.node import Node
from custom_interface.srv import GetBestMove, ValidateMove
from stockfish import Stockfish
import chess

PIECE_TYPE_MAP = {
    "Pawn": chess.PAWN,
    "Knight": chess.KNIGHT,
    "Bishop": chess.BISHOP,
    "Rook": chess.ROOK,
    "Queen": chess.QUEEN,
    "King": chess.KING
}

COLOR_MAP = {
    "White": chess.WHITE,
    "Black": chess.BLACK
}


class ChessAIServer(Node):

    def __init__(self):
        super().__init__('chess_ai_server')
        self.get_best_move_srv = self.create_service(GetBestMove, 'get_best_move', self.get_best_move_callback)
        self.validate_move_srv = self.create_service(ValidateMove, 'validate_move', self.validate_move_callback)
        self.setup_stockfish()
        self.setup_chess()
        self.get_logger().info('Chess AI Server node is ready.')

    def setup_stockfish(self):
        stockfish_binary_path = "/robotics_final_project/ros2_ws/install/chess_ai_pkg/share/chess_ai_pkg/stockfish_engine/stockfish-ubuntu-x86-64-avx2"
        self.get_logger().info(f"Loading Stockfish from: {stockfish_binary_path}")
        try:
            self.stockfish = Stockfish(path=stockfish_binary_path)
            self.stockfish.set_depth(23)
            self.get_logger().info("Stockfish engine initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Stockfish: {e}")

    def setup_chess(self):
        self.board = chess.Board()

    def reset_board(self):
        self.board = chess.Board()
        self.stockfish.set_fen_position(self.board.fen())

    def push_player_move(self, move_uci):
        self.board.push_uci(move_uci)
        self.get_logger().info(f"Player move pushed: {move_uci}")

    def get_ai_move(self):
        self.stockfish.set_fen_position(self.board.fen())
        ai_move = self.stockfish.get_best_move()
        self.board.push_uci(ai_move)
        self.get_logger().info(f"AI move: {ai_move}")
        self.get_logger().info(f"FEN: {self.board.fen()}")
        return ai_move

    def evaluate_board_state(self):
        if self.board.is_checkmate():
            self.get_logger().info("CHECKMATE detected!")
            outcome = self.board.outcome()
            result = outcome.result()
            self.get_logger().info(f"Game result: {result}")
            if result == "1-0":
                return "CHECKMATE_WHITE_WINS"
            elif result == "0-1":
                return "CHECKMATE_BLACK_WINS"
            else:
                return "CHECKMATE"

        if self.board.is_stalemate():
            self.get_logger().info("Stalemate detected!")
            return "STALEMATE"

        if self.board.is_insufficient_material():
            self.get_logger().info("Draw — insufficient material.")
            return "DRAW_INSUFFICIENT_MATERIAL"

        if self.board.can_claim_threefold_repetition():
            self.get_logger().info("Draw — threefold repetition possible.")
            return "DRAW_REPETITION"

        if self.board.is_check():
            self.get_logger().info("CHECK!")

        return None

    def get_mate_in_n(self):
        eval_info = self.stockfish.get_evaluation()
        if eval_info["type"] == "mate":
            mate_value = eval_info["value"]
            self.get_logger().info(f"Stockfish reports mate in {mate_value}")
            return f"MATE_IN_{mate_value}"
        return None

    def validate_human_move(self, scanned_dict):
        for move in self.board.legal_moves:
            test_board = self.board.copy()
            test_board.push(move)

            match = True
            for square in chess.SQUARES:
                square_name = chess.square_name(square)
                expected_piece = test_board.piece_at(square)
                scanned_piece = scanned_dict.get(square_name)

                if expected_piece is None and scanned_piece is None:
                    continue
                if expected_piece is None or scanned_piece is None:
                    match = False
                    break
                if expected_piece != chess.Piece(PIECE_TYPE_MAP[scanned_piece["type"]], COLOR_MAP[scanned_piece["color"]]):
                    match = False
                    break

            if match:
                return move.uci()

        return "INVALID"

    def get_best_move_callback(self, request, response):
        players_move = request.players_move

        if players_move == "START_GAME":
            self.reset_board()
            ai_move = self.get_ai_move()
            state = self.evaluate_board_state()
            if state:
                response.best_move = state
                return response
            mate_prediction = self.get_mate_in_n()
            if mate_prediction:
                self.get_logger().info(f"Mate prediction: {mate_prediction}")
            response.best_move = ai_move
            return response

        self.push_player_move(players_move)
        state = self.evaluate_board_state()
        if state:
            response.best_move = state
            return response

        ai_move = self.get_ai_move()
        state = self.evaluate_board_state()
        if state:
            response.best_move = state
            return response

        mate_prediction = self.get_mate_in_n()
        if mate_prediction:
            self.get_logger().info(f"Mate prediction: {mate_prediction}")

        response.best_move = ai_move
        return response

    def validate_move_callback(self, request, response):
        scanned_dict = json.loads(request.board_json)
        response.validated_move = self.validate_human_move(scanned_dict)
        self.get_logger().info(f"Validated move: {response.validated_move}")
        return response

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    chess_ai_server = ChessAIServer()
    try:
        rclpy.spin(chess_ai_server)
    except KeyboardInterrupt:
        pass
    chess_ai_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()