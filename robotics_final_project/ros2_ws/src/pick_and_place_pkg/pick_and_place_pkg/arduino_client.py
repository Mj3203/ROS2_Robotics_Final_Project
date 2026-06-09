import time
import threading

try:
    import serial
    from serial.tools import list_ports
    SERIAL_AVAILABLE = True
except Exception:
    serial = None
    list_ports = None
    SERIAL_AVAILABLE = False

ARDUINO_BOOT_DELAY = 0.2
RECONNECT_DELAY = 2.0
READER_SLEEP_INTERVAL = 0.05


# ArduinoSerialClient manages all serial communication with the Arduino.
# It handles two things simultaneously:
# 1. Continuously listening for incoming messages from the Arduino (button presses)
# 2. Sending commands to the Arduino on demand (solenoid on/off)
#
# Since both of these happen at the same time, a threading lock is used to ensure
# only one thread accesses the serial connection at a time. Think of it like a
# walkie talkie — only one person can talk at a time, otherwise the messages get garbled.
#
# The reader runs in a background thread (read_loop) that never stops while the
# node is running. If the Arduino disconnects or the USB cable gets bumped, it
# automatically tries to reconnect every 2 seconds.
#
# Public methods:
# - send_command(line) : sends a command string to the Arduino e.g. "sol on"
# - stop_serial_connection() : cleanly shuts down the serial connection
class ArduinoSerialClient:

    # Stores configuration, initializes connection state, and starts the reader thread if pyserial is available.
    def __init__(self, node, port="/dev/ttyUSB0", baud=115200, timeout=1.0, button_callback=None):
        self.node = node
        self.port = port
        self.baud = int(baud)
        self.timeout = float(timeout)
        self.button_callback = button_callback
        self.connection = None
        self.reader_thread = None
        self.running = False
        self.lock = threading.Lock()

        if not SERIAL_AVAILABLE:
            self.node.get_logger().warn("pyserial not available. Arduino disabled.")
            return

        self.start_reader_thread()

    # Starts the background thread that runs the serial read loop.
    def start_reader_thread(self):
        self.running = True
        self.reader_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.reader_thread.start()

    # Opens the serial connection to the Arduino, returning the connection or None on failure.
    def connect_to_arduino(self):
        try:
            connection = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(ARDUINO_BOOT_DELAY)
            self.node.get_logger().info(f"Connected to Arduino on {self.port} @ {self.baud}")
            return connection
        except Exception as e:
            self.node.get_logger().warn(f"Failed to connect to Arduino on {self.port}: {e}")
            return None

    # Returns True if a raw serial line is a button-press message.
    def is_button_pressed(self, raw):
        return raw.upper().startswith("BUTTON")

    # Background loop that connects, reads incoming lines, dispatches button presses, and reconnects on error.
    def read_loop(self):
        while self.running:
            with self.lock:
                if self.connection is None:
                    self.connection = self.connect_to_arduino()
                    if self.connection is None:
                        time.sleep(RECONNECT_DELAY)
                        continue

            try:
                with self.lock:
                    has_data = self.connection.in_waiting

                if has_data:
                    with self.lock:
                        raw = self.connection.readline().decode(errors='ignore').strip()

                    if raw:
                        self.node.get_logger().debug(f"[Arduino] {raw}")
                        if self.is_button_pressed(raw):
                            self.node.get_logger().info(f"Button press detected: {raw}")
                            if self.button_callback:
                                self.button_callback()
                else:
                    time.sleep(READER_SLEEP_INTERVAL)

            except Exception as e:
                self.node.get_logger().warn(f"Arduino read error: {e}")
                with self.lock:
                    try:
                        self.connection.close()
                    except Exception:
                        pass
                    self.connection = None
                time.sleep(RECONNECT_DELAY)

    # Sends a command string to the Arduino, returning True on success and dropping the connection on failure.
    def send_command(self, line: str):
        if not SERIAL_AVAILABLE:
            return False

        with self.lock:
            if self.connection is None:
                return False
            try:
                msg = (line.strip() + '\n').encode()
                self.connection.write(msg)
                self.connection.flush()
                return True
            except Exception:
                try:
                    self.connection.close()
                except Exception:
                    pass
                self.connection = None
                return False

    # Stops the reader loop and closes the serial connection.
    def stop_serial_connection(self):
        self.running = False
        with self.lock:
            if self.connection:
                try:
                    self.connection.close()
                except Exception:
                    pass
            self.connection = None
