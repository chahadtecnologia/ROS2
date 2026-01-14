#!/usr/bin/env python3
import rclpy
import requests
from rclpy.node import Node
from temperature_collector_interfaces.msg import TemperatureCollector

class TempCollectorNode(Node):
    def __init__(self):
        super().__init__("temp_collector_node")
        self.temp_collector_publisher_ = self.create_publisher(TemperatureCollector, "/temp_collector_msg", 10)
        self.timer_ = self.create_timer(10.0, self.extract_temp_collector)
        self.get_logger().info("Temperature Collector publisher has been started...")

        self.declare_parameter("city1", "São Paulo")
        self.declare_parameter("city2", "Santo André")
        self.declare_parameter("city3", "Campinas")

    def get_weather_data(self):

        self.city1_name_ = self.get_parameter("city1").value
        self.city2_name_ = self.get_parameter("city2").value
        self.city3_name_ = self.get_parameter("city3").value

        api_key = "<API_TOKEN>"
        cities = [self.city1_name_, self.city2_name_, self.city3_name_]
        results = {}

        for city in cities:
            url = f"<WEATHER_MAP_URL>"
            try:
                response = requests.get(url)
                data = response.json()
                if response.status_code == 200:
                    results[city] = data['main']['temp']
                else:
                    self.get_logger().error(f"Error in {city}: {data.get('message')}")
            except Exception as e:
                self.get_logger().error(f"Conection error in {city}: {e}")
        
        return results

    def extract_temp_collector(self):
        temperatures = self.get_weather_data()

        self.city1_name_ = self.get_parameter("city1").value
        self.city2_name_ = self.get_parameter("city2").value
        self.city3_name_ = self.get_parameter("city3").value
        
        if temperatures:
            msg = TemperatureCollector()
            msg.temperature_city1 = temperatures.get(self.city1_name_, 0.0)
            msg.temperature_city2 = temperatures.get(self.city2_name_, 0.0)
            msg.temperature_city3 = temperatures.get(self.city3_name_, 0.0)
            
            self.temp_collector_publisher_.publish(msg)
            self.get_logger().info("Temperature Collected by Message Interface: " + self.city1_name_ + ": " + str(msg.temperature_city1) + " | " 
                                                             + self.city2_name_ + ": " + str(msg.temperature_city2) + " | "
                                                             + self.city3_name_ + ": " + str(msg.temperature_city3))

def main(args=None):
    rclpy.init(args=args)
    node = TempCollectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
