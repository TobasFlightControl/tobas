import rclpy
from tobas_rclpy.utils import init_node
from tobas_fake_publishers.fake_gnss_publisher import FakeGNSSPublisher

if __name__ == "__main__":
    init_node()
    node = FakeGNSSPublisher()
    rclpy.spin()
