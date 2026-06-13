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

PIECE_NAME_MAP = {
    chess.PAWN: "Pawn",
    chess.KNIGHT: "Knight",
    chess.BISHOP: "Bishop",
    chess.ROOK: "Rook",
    chess.QUEEN: "Queen",
    chess.KING: "King",
}


class ChessAIServer(Node):

    # Initializes the node, creates the chess services, and runs all setup steps.
    def __init__(self):
        super().__init__('chess_ai_server')
        self.get_best_move_srv = self.create_service(GetBestMove, 'get_best_move', self.get_best_move_callback)
        self.validate_move_srv = self.create_service(ValidateMove, 'validate_move', self.validate_move_callback)
        self.setup_stockfish()
        self.setup_chess()
        self.get_logger().info('Chess AI Server node is ready.')

    # Loads the bundled Stockfish engine binary and sets its search depth.
    def setup_stockfish(self):
        stockfish_binary_path = "/robotics_final_project/ros2_ws/install/chess_ai_pkg/share/chess_ai_pkg/stockfish_engine/stockfish-ubuntu-x86-64-avx2"
        self.get_logger().info(f"Loading Stockfish from: {stockfish_binary_path}")
        try:
            self.stockfish = Stockfish(path=stockfish_binary_path)
            self.stockfish.set_depth(10)
            self.get_logger().info("Stockfish engine initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Stockfish: {e}")

    # Creates the authoritative python-chess board that tracks game state.
    def setup_chess(self):
        self.board = chess.Board()

    # Resets the board to the starting position and syncs Stockfish to it.
    def reset_board(self):
        self.board = chess.Board()
        self.stockfish.set_fen_position(self.board.fen())

    # Returns the current legal moves with pawn promotions filtered out.
    def get_restricted_legal_moves(self):
        return [move for move in self.board.legal_moves if move.promotion is None]

    # Asks Stockfish for the best non-promotion move, records whether it is a capture, and applies it to the board.
    def get_ai_move_with_capture(self):
        self.stockfish.set_fen_position(self.board.fen())
        allowed_ucis = [move.uci() for move in self.get_restricted_legal_moves()]
        ranked_moves = self.stockfish.get_top_moves(self.board.legal_moves.count())
        ai_move_uci = next((m["Move"] for m in ranked_moves if m["Move"] in allowed_ucis), allowed_ucis[0])
        ai_move = chess.Move.from_uci(ai_move_uci)
        is_capture = self.board.is_capture(ai_move)
        self.board.push(ai_move)
        self.get_logger().info(f"AI move: {ai_move_uci} (capture={is_capture})")
        self.get_logger().info(f"FEN: {self.board.fen()}")
        return ai_move_uci, is_capture

    # Applies the human player's move to the board.
    def push_player_move(self, move_uci):
        self.board.push_uci(move_uci)
        self.get_logger().info(f"Player move pushed: {move_uci}")

    # Checks the board for checkmate, stalemate, draw, or check and returns a status string.
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
            self.get_logger().info("Draw - insufficient material.")
            return "DRAW_INSUFFICIENT_MATERIAL"

        if self.board.can_claim_threefold_repetition():
            self.get_logger().info("Draw - threefold repetition possible.")
            return "DRAW_REPETITION"

        if self.board.is_check():
            self.get_logger().info("CHECK!")
            return "CHECK"

        return ""

    # Asks Stockfish whether the current position is a forced mate and returns the mate count.
    def get_mate_in_n(self):
        eval_info = self.stockfish.get_evaluation()
        if eval_info["type"] == "mate":
            mate_value = eval_info["value"]
            self.get_logger().info(f"Stockfish reports mate in {mate_value}")
            return f"MATE_IN_{mate_value}"
        return None

    # Compares the scanned board against the expected board to explain why a move was invalid.
    def get_invalid_reason(self, scanned_dict):
        for square in chess.SQUARES:
            square_name = chess.square_name(square)
            expected_piece = self.board.piece_at(square)
            scanned_piece = scanned_dict.get(square_name)

            if expected_piece is None and scanned_piece is None:
                continue

            if expected_piece is not None and scanned_piece is None:
                piece_name = PIECE_NAME_MAP.get(expected_piece.piece_type, "Piece")
                return f"{piece_name} {square_name} invalid"

            if expected_piece is None and scanned_piece is not None:
                return f"{scanned_piece['type']} {square_name} invalid"

            expected = chess.Piece(PIECE_TYPE_MAP[scanned_piece["type"]], COLOR_MAP[scanned_piece["color"]])
            if expected_piece != expected:
                piece_name = PIECE_NAME_MAP.get(expected_piece.piece_type, "Piece")
                return f"{piece_name} {square_name} invalid"

        return "Move invalid"

    # Finds the legal move whose resulting position matches the scanned board, or reports it as invalid.
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
                return move.uci(), ""

        return "INVALID", self.get_invalid_reason(scanned_dict)

    # Service handler that starts a game or processes a player move and returns the AI's reply.
    def get_best_move_callback(self, request, response):
        players_move = request.players_move

        if players_move == "START_GAME":
            self.reset_board()
            ai_move, is_capture = self.get_ai_move_with_capture()
            game_status = self.evaluate_board_state()
            response.best_move = ai_move
            response.game_status = game_status
            response.is_capture = is_capture
            if game_status in ("CHECKMATE_WHITE_WINS", "CHECKMATE_BLACK_WINS", "CHECKMATE", "STALEMATE", "DRAW_INSUFFICIENT_MATERIAL", "DRAW_REPETITION"):
                return response
            mate_prediction = self.get_mate_in_n()
            if mate_prediction:
                self.get_logger().info(f"Mate prediction: {mate_prediction}")
            return response

        self.push_player_move(players_move)
        game_status = self.evaluate_board_state()
        if game_status in ("CHECKMATE_WHITE_WINS", "CHECKMATE_BLACK_WINS", "CHECKMATE", "STALEMATE", "DRAW_INSUFFICIENT_MATERIAL", "DRAW_REPETITION"):
            response.best_move = ""
            response.game_status = game_status
            response.is_capture = False
            return response

        ai_move, is_capture = self.get_ai_move_with_capture()
        game_status = self.evaluate_board_state()
        response.best_move = ai_move
        response.game_status = game_status
        response.is_capture = is_capture
        if game_status in ("CHECKMATE_WHITE_WINS", "CHECKMATE_BLACK_WINS", "CHECKMATE", "STALEMATE", "DRAW_INSUFFICIENT_MATERIAL", "DRAW_REPETITION"):
            return response
        mate_prediction = self.get_mate_in_n()
        if mate_prediction:
            self.get_logger().info(f"Mate prediction: {mate_prediction}")
        return response

    # Service handler that validates a scanned board state against the legal moves.
    def validate_move_callback(self, request, response):
        scanned_dict = json.loads(request.board_json)
        validated_move, invalid_reason = self.validate_human_move(scanned_dict)
        response.validated_move = validated_move
        response.invalid_reason = invalid_reason
        self.get_logger().info(f"Validated move: {validated_move}")
        if invalid_reason:
            self.get_logger().warning(f"Invalid reason: {invalid_reason}")
        return response

    # Cleans up and shuts the node down.
    def destroy_node(self):
        super().destroy_node()


# Initializes ROS, spins the node until interrupted, then shuts everything down.
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
