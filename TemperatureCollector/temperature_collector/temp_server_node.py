#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from temperature_collector_interfaces.msg import TemperatureCollector
 
class TempServerNode(Node):
    def __init__(self):
        super().__init__("temp_server_node")
        self.current_msg = None
        self.temp_server_subscription_ = self.create_subscription(TemperatureCollector, "/temp_collector_msg", self.listener_callback, 10)
        self.temp_server_publisher_ = self.create_publisher(TemperatureCollector, "/temp_sender_msg", 10)
        self.timer_ = self.create_timer(10.0, self.callback_temp_server)

        self.declare_parameter("city1", "São Paulo")
        self.declare_parameter("city2", "Santo André")
        self.declare_parameter("city3", "Campinas")

        self.get_logger().info("Temperature server started. Collecting every 10s...")

    def listener_callback(self, msg: TemperatureCollector):
        self.current_msg = msg

    def callback_temp_server(self):
        if self.current_msg is not None:
            self.city1_name_ = self.get_parameter("city1").value
            self.city2_name_ = self.get_parameter("city2").value
            self.city3_name_ = self.get_parameter("city3").value
            
            self.get_logger().info("Temperature Collected: " + self.city1_name_ + ": " + str(self.current_msg.temperature_city1) 
                                                             + " | " + self.city2_name_ + ": " + str(self.current_msg.temperature_city2) 
                                                             + " | " + self.city3_name_ + ": " + str(self.current_msg.temperature_city3))
            
            self.get_logger().info(f"Publishing temperatures to the clients...")
            self.temp_server_publisher_.publish(self.current_msg)

        else:
            self.get_logger().info("No data received yet...")

 
def main(args=None):
    rclpy.init(args=args)
    node = TempServerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()