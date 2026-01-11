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

    def get_weather_data(self):
        api_key = "<API_TOKEN>"
        cities = ["São Paulo", "Santo André", "Campinas"]
        results = {}

        for city in cities:
            url = f"<OPEN_WEATHER_MAP_ENDPOINT>"
            try:
                response = requests.get(url)
                data = response.json()
                if response.status_code == 200:
                    results[city] = data['main']['temp']
                else:
                    self.get_logger().error(f"Erro em {city}: {data.get('message')}")
            except Exception as e:
                self.get_logger().error(f"Erro de conexão em {city}: {e}")
        
        return results

    def extract_temp_collector(self):
        temperatures = self.get_weather_data()
        
        if temperatures:
            msg = TemperatureCollector()
            msg.temperature_saopaulo = temperatures.get("São Paulo", 0.0)
            msg.temperature_santoandre = temperatures.get("Santo André", 0.0)
            msg.temperature_campinas = temperatures.get("Campinas", 0.0)
            
            self.temp_collector_publisher_.publish(msg)
            self.get_logger().info(f"Temperature Collected: São Paulo: {msg.temperature_saopaulo} | Santo André: {msg.temperature_santoandre} | Campinas: {msg.temperature_campinas}")

def main(args=None):
    rclpy.init(args=args)
    node = TempCollectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
