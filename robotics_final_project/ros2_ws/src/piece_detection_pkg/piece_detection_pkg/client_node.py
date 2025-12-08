import sys 

import rclpy
from rclpy.node import Node
from custom_interface.srv import GetBoardState

class ClientAsync(Node):
    def __init__(self):
        super().__init__("client_async")
        timeout_sec = 5.0
        self.client = self.create_client(GetBoardState, "get_board_state")
        while not self.client.wait_for_service(timeout_sec):
            self.get_logger().info("service not available, waiting again ...")
    
    def send_request(self):
        request = GetBoardState.Request()
        request.request_message = sys.argv[0]
        self.future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    client = ClientAsync()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(
                    f"Service call failed {e}"
                )
            else:
                client.get_logger().info(
                    f"Result of addition is {response.board_json}"
                )
            break
        client.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()