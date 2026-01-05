#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
 
class MagusRobotSub2(Node):
    def __init__(self):
        super().__init__("magus_robot_sub2")
        self.declare_parameter("name", "Magus 2")
        self.robot_name_ = self.get_parameter("name").value
        self.subscriber_ = self.create_subscription(
            String, "/magus_topic", self.callback_magus_news, 10)
        self.get_logger().info("Receiving data from " + self.robot_name_)
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Data from " + self.robot_name_ + " to the Magus Server"
        self.get_logger().info("Receiving data from " + self.robot_name_)
 
    def callback_magus_news(self, msg: String):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = MagusRobotSub2()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()