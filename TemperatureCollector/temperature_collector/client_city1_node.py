#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from temperature_collector_interfaces.msg import TemperatureCollector 
from temperature_collector_interfaces.srv import TemperatureService
 
class ClientCity1Node(Node):
    def __init__(self):
        super().__init__("client_city1_node")
        self.current_msg = None
        self.declare_parameter("city1", "São Paulo")
        self.temp_server_subscription_ = self.create_subscription(TemperatureCollector, "temp_sender_msg", self.listener_callback, 10)
        self.client_ = self.create_client(TemperatureService, "temp_sender_srv")
        self.timer_ = self.create_timer(10.0, self.callback_temp_server)

        while not self.client_.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Server is not ready, waiting...')
        
        self.timer = self.create_timer(10.0, self.send_request)
 
    def listener_callback(self, msg: TemperatureCollector):
        self.current_msg = msg

    def callback_temp_server(self):
        if self.current_msg is not None:
            self.city1_name_ = self.get_parameter("city1").value
            self.get_logger().info("Temperature Collected by Message Interface: " + self.city1_name_ + ": " + str(self.current_msg.temperature_city1))
            pass
        else:
            self.get_logger().info("No data received yet...")

    def send_request(self):
        req = TemperatureService.Request()
        req.city_name = self.get_parameter("city1").value
        
        self.future = self.client_.call_async(req)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.city1_name_ = self.get_parameter("city1").value
                self.get_logger().info("Temperature Collected by Service Interface: " + self.city1_name_ + ": " + str(response.temperature))
            else:
                self.get_logger().error("Error to collect temperature or city is not registered.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
 
def main(args=None):
    rclpy.init(args=args)
    node = ClientCity1Node()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()