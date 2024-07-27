import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
import asyncio
from websockets.server import serve
import threading

class SensingNetwork(Node):

    def __init__(self, size):
        super().__init__('sensing_network')
        self.publisher_ = self.create_publisher(Int8MultiArray, 'contacts', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.data = [0] * size

        # Start the asyncio loop in a separate thread
        self.loop_thread = threading.Thread(target=self.start_async_loop, daemon=True)
        self.loop_thread.start()

    def timer_callback(self):
        msg = Int8MultiArray()
        msg.data = self.data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    async def onMessage(self, websocket):
        self.get_logger().info("Client connected, IP: %s" % str(websocket.local_address))

        async for message in websocket:
            try:
                data = message.split(",")

                self.data[int(data[0])] = int(data[1])
            except Exception as e:
                self.get_logger().error("Couldn't parse websocket input: %s" % e)

    async def asyncRoutine(self):
        async with serve(self.onMessage, "0.0.0.0", 8765):
            await asyncio.Future()  # run forever

    def start_async_loop(self):
        asyncio.run(self.asyncRoutine())

def main(args=None):
    rclpy.init(args=args)

    sensing_network = SensingNetwork(10)

    try:
        rclpy.spin(sensing_network)
    finally:
        # Destroy the node explicitly
        sensing_network.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
