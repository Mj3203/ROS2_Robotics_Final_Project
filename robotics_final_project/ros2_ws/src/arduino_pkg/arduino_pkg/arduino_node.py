import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from custom_interface.srv import SetSolenoid, SetMotor
from arduino_pkg.arduino_client import ArduinoSerialClient, SERIAL_AVAILABLE, list_ports


class ArduinoNode(Node):

    # Initializes the node, declares parameters, sets up the button publisher and Arduino client, and hosts the solenoid/motor services.
    def __init__(self):
        super().__init__("arduino_node")
        self.declare_parameter("arduino_enabled", True)
        self.declare_parameter("arduino_port", "/dev/ttyUSB0")
        self.declare_parameter("arduino_baud", 115200)
        self.declare_parameter("auto_detect_arduino", True)
        self.button_pub = self.create_publisher(Empty, "button_pressed", 10)
        self.setup_arduino()
        self.set_solenoid_srv = self.create_service(SetSolenoid, "set_solenoid", self.handle_set_solenoid)
        self.set_motor_srv = self.create_service(SetMotor, "set_motor", self.handle_set_motor)
        self.get_logger().info("ArduinoNode is ready.")

    # Connects to the Arduino serial client, auto-detecting the port unless disabled, and wires button presses to the publisher.
    def setup_arduino(self):
        self.arduino = None
        enabled = self.get_parameter("arduino_enabled").get_parameter_value().bool_value
        if not enabled:
            self.get_logger().info("Arduino disabled via parameter.")
            return

        port = self.get_parameter("arduino_port").get_parameter_value().string_value
        baud = self.get_parameter("arduino_baud").get_parameter_value().integer_value

        if SERIAL_AVAILABLE and list_ports is not None:
            try:
                byid_dir = "/dev/serial/by-id"
                if os.path.isdir(byid_dir):
                    entries = sorted(os.listdir(byid_dir))
                    if entries:
                        port = os.path.join(byid_dir, entries[0])
                        self.get_logger().info(f"Auto-detected Arduino by-id: {port}")
                else:
                    for p in list_ports.comports():
                        desc = (p.description or "").lower()
                        if "arduino" in desc or "usb serial" in desc or "ch341" in desc or "ftdi" in desc:
                            port = p.device
                            self.get_logger().info(f"Auto-detected Arduino: {port}")
                            break
            except Exception as e:
                self.get_logger().debug(f"Auto-detect failed: {e}")

        try:
            self.arduino = ArduinoSerialClient(node=self, port=port, baud=baud, button_callback=self.publish_button_press)
            self.get_logger().info(f"Arduino client enabled on {port} @ {baud}")
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize Arduino client: {e}")
            self.arduino = None

    # Publishes an Empty message on button_pressed whenever the Arduino reports a button press.
    def publish_button_press(self):
        self.button_pub.publish(Empty())

    # Service handler that turns the solenoid on or off from the request state and reports whether the serial write succeeded.
    def handle_set_solenoid(self, request, response):
        command = "sol on" if request.state else "sol off"
        response.success = self.send_arduino_command(command)
        return response

    # Service handler that turns the motor on or off from the request state and reports whether the serial write succeeded.
    def handle_set_motor(self, request, response):
        command = "motor on" if request.state else "motor off"
        response.success = self.send_arduino_command(command)
        return response

    # Sends a command string to the Arduino and returns whether the write succeeded, skipping if no client is available.
    def send_arduino_command(self, command):
        if self.arduino is None:
            self.get_logger().warn(f"Arduino command skipped — no Arduino client available: {command}")
            return False
        ok = self.arduino.send_command(command)
        if ok:
            self.get_logger().info(f"Arduino: {command}")
        else:
            self.get_logger().warn(f"Arduino command failed: {command}")
        return ok

    # Closes the Arduino connection and shuts the node down.
    def destroy_node(self):
        if self.arduino:
            self.arduino.stop_serial_connection()
        super().destroy_node()


# Initializes ROS, spins the node until interrupted, then shuts everything down.
def main(args=None):
    rclpy.init(args=args)
    arduino_node = ArduinoNode()
    try:
        rclpy.spin(arduino_node)
    except KeyboardInterrupt:
        pass
    arduino_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
