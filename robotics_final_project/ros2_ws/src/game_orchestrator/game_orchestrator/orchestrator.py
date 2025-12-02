import rclpy
from rclpy.node import Node

from game_state.srv import GetBoardState   # service you already created

# Additional imports needs to be added

import json
import time


class Orchestrator(Node):
    def __init__(self):
        super().__init__('orchestrator')

        self.get_logger().info("Orchestrator node started.")

        # === SERVICE CLIENT: GetBoardState (already implemented) ===
        self.board_state_client = self.create_client(GetBoardState, 'get_board_state')

        while not self.board_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for get_board_state service...")

        # === SUBSCRIBER: listen for human button press (future) ===
        # self.button_sub = self.create_subscription(Bool, 'button_pressed', self.button_callback, 10)

        # === ACTION CLIENT: send robot moves (future) ===
        # self.robot_action_client = ActionClient(self, ExecuteMove, 'execute_move')

        # === SERVICE CLIENT: chess engine (future) ===
        # self.ai_client = self.create_client(GetBestMove, 'get_ai_move')

        # Internal state
        self.button_pressed = False

        # Start main loop timer (runs every second)
        self.timer = self.create_timer(1.0, self.main_loop)


    # ==========================================================
    # (FUTURE) callback for human button press
    # ==========================================================
    # def button_callback(self, msg):
    #     self.get_logger().info("Human move button pressed.")
    #     self.button_pressed = True

    # ==========================================================
    # Helper: call GetBoardState service and return dict
    # ==========================================================
    def request_board_state(self, message: str):
        req = GetBoardState.Request()
        req.request_message = message

        future = self.board_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("Board state service failed.")
            return {}

        board_json = future.result().board_json
        return json.loads(board_json)


    # ==========================================================
    # Helper: compare two board dictionaries
    # ==========================================================
    def detect_move(self, before, after):
        """
        Return a tuple like:
        ("e2", "e4", "white_pawn")
        """
        #previous_board_state =
        #current_board_state =

        #prev_key = None
        #for key in previous_board_state:
        #    if key not in current_board_state:
        #        print(f'{key} is not present in the current board state')
        #        prev_key = key
        #
        #curr_key = None
        #for key in current_board_state:
        #    if key not in previous_board_state:
        #        print(f'{key} is not present in the previous board state')
        #        curr_key = key

        #move_made = prev_key + curr_key
        #return
        
    # ==========================================================
    # Main loop runs every 1 second
    # ==========================================================
    def main_loop(self):
        """
        HIGH-LEVEL FLOW:
            1. Ask for board → dict_before
            2. Wait for button press (HUMAN MOVE)
            3. Ask for board → dict_after
            4. Detect move
            5. Send move to chess AI (future)
            6. Send engine move to robot (future)
        """

        # ----------------------------
        # 1. Get board before human move
        # ----------------------------
        self.get_logger().info("Requesting board BEFORE human move...")
        before = self.request_board_state("Board before human move")

        self.get_logger().info(f"Board BEFORE:\n{before}")

        # ----------------------------
        # 2. Wait for human move (placeholder)
        # ----------------------------
        self.get_logger().info("Waiting for human move...")

        # (FOR NOW) Just sleep 5 seconds instead of waiting for a button
        # Later, replace this with button_pressed logic
        time.sleep(5)

        # ----------------------------
        # 3. Get board after human move
        # ----------------------------
        self.get_logger().info("Requesting board AFTER human move...")
        after = self.request_board_state("Board after human move")

        self.get_logger().info(f"Board AFTER:\n{after}")

        # ----------------------------
        # 4. Detect human move
        # ----------------------------
        move = self.detect_move(before, after)

        if move is None:
            self.get_logger().warn("Move detection failed.")
            return

        src, dst, piece = move
        self.get_logger().info(f"Detected human move: {piece} {src} → {dst}")

        # ----------------------------
        # 5. Call chess AI (future)
        # ----------------------------
        """
        req = GetBestMove.Request()
        req.human_move = f"{src}{dst}"
        future = self.ai_client.call_async(req)
        ...
        ai_move = future.result().best_move
        """

        # Just placeholder
        ai_move = "e7e5"
        self.get_logger().info(f"AI move would be: {ai_move}")

        # ----------------------------
        # 6. Command robot to execute AI move (future)
        # ----------------------------
        """
        goal = ExecuteMove.Goal()
        goal.move = ai_move
        self.robot_action_client.send_goal_async(goal)
        """
        self.get_logger().info("Robot would now perform the move.")

        # END OF LOOP; will run again automatically


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
