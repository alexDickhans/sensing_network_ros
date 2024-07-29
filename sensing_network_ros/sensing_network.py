import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import asyncio
from websockets.server import serve
import threading

# Basic configuration, set the number of points to the TOTAL number of sensor points on the robot
NUM_POINTS = 140
# Port for the websocket server to listen to, must be the same as the port that is set in the arduino code for this to work properly
# Default: 8765
PORT = 8765
# Set the timer period for the publisher node
# default 0.01 seconds
TIMER_PERIOD = 0.01 # seconds

# Simple Publisher Class for the touch points
class SensingNetwork(Node):

    def __init__(self, size, port, timer_period):
        super().__init__('sensing_network')

        # Initialize the port and data
        self.port = port
        self.data = [0.0] * size

        # Create ROS publisher and callback timer
        self.publisher = self.create_publisher(Float32MultiArray, 'contact_list', 1)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Start the asyncio loop in a separate thread
        self.loop_thread = threading.Thread(target=self.start_async_loop, daemon=True)
        self.loop_thread.start()

    # The timer callback to be used by the ROS timer
    def timer_callback(self):
        # Create and send the new message
        msg = Float32MultiArray()
        msg.data = self.data
        self.publisher.publish(msg)

    # Async routine to be called on new messages recieved from the websocket
    async def on_connection(self, websocket):
        self.get_logger().info("Client connected, IP: %s" % str(websocket.local_address))

        # Loop through each message recieved by the
        async for message in websocket:
            try:
                # Message format:
                # [index],[new value]\n[index][new value]

                # For newline-delimted, multi touchpoint update messages
                split_message = message.split("\n")

                # Loop through each individual sensor update
                for updates in split_message:
                    # Split on the comma to seperate value and index
                    split_update = updates.split(",")

                    # Sanity check list length before changing array
                    if (len(split_update) == 2):
                        self.data[int(split_update[0])] = float(split_update[1])

            # Catch reading exceptions without closing the connection
            except Exception as e:
                self.get_logger().error("Couldn't parse websocket input: %s" % e)

    # Run the async routine to start the websocket server on the localhost with the desired port
    async def async_routine(self):
        async with serve(self.on_connection, "0.0.0.0", self.port):
            await asyncio.Future()  # run forever

    # Start the async routine in a call that can be started with the threading API
    # This is so rclpy can spin while the webserver does work
    def start_async_loop(self):
        asyncio.run(self.async_routine())

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)

    # Create the node object
    sensing_network = SensingNetwork(NUM_POINTS, PORT, TIMER_PERIOD)

    # Spin until the ROS node is done, then shutdown cleanly
    try:
        rclpy.spin(sensing_network)
    finally:
        # Destroy the node explicitly
        sensing_network.destroy_node()
        rclpy.shutdown()

# Run main
if __name__ == '__main__':
    main()
