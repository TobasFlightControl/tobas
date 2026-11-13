import rclpy
from rclpy.node import Node


class UserNode(Node):
    def __init__(self) -> None:
        super().__init__("user_python_node")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UserNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
