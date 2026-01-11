#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from temperature_collector_interfaces.msg import TemperatureCollector 
 
class ClientCity1Node(Node):
    def __init__(self):
        super().__init__("client_city1_node")
        self.current_msg = None
        self.declare_parameter("city1", "São Paulo")
        self.temp_server_subscription_ = self.create_subscription(TemperatureCollector, "temp_sender_msg", self.listener_callback, 10)
        self.timer_ = self.create_timer(10.0, self.callback_temp_server)
 
    def listener_callback(self, msg: TemperatureCollector):
        self.current_msg = msg

    def callback_temp_server(self):
        if self.current_msg is not None:
            self.city1_name_ = self.get_parameter("city1").value
            self.get_logger().info("Temperature Collected: " + self.city1_name_ + ": " + str(self.current_msg.temperature_city1))
            pass
        else:
            self.get_logger().info("No data received yet...")
 
def main(args=None):
    rclpy.init(args=args)
    node = ClientCity1Node()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()