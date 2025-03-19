import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from tobas_command_msgs.msg import PosVelYaw
from tobas_mission_msgs.action import Takeoff


class CommandSquareTrajectoryNode(Node):
    ALTITUDE = 3.0  # [m]
    SIDE_LENGTH = 5.0  # [m]
    INTERVAL = 5.0  # [s]

    def __init__(self) -> None:
        super().__init__("command_square_trajectory")

        # 離陸アクションクライアントの作成
        self._takeoff_client = ActionClient(self, Takeoff, "takeoff_action")

        # コマンドのパブリッシャーを作成
        self._command_pub = self.create_publisher(PosVelYaw, "command/pos_vel_yaw", 1)

    def run(self) -> None:
        # アクションサーバーが起動するのを待つ
        self.get_logger().info("Waiting for takeoff action server.")
        if not self._takeoff_client.wait_for_server():
            self.get_logger().error("Failed to connect to takeoff action server.")
            return

        # アクションゴールを作成
        takeoff_goal = Takeoff.Goal()
        takeoff_goal.target_altitude = self.ALTITUDE
        takeoff_goal.duration = self.INTERVAL

        # アクションを実行
        takeoff_result: Takeoff.Result = self._takeoff_client.send_goal(takeoff_goal)

        # アクションの結果を取得
        self._takeoff_client
        if takeoff_result.success:
            self.get_logger().error(f"Takeoff action failed: {takeoff_result.message}")
            return

        # 正方形の頂点を指令し続ける
        while rclpy.ok():
            # 頂点1
            command = PosVelYaw()
            command.pos.x = self.SIDE_LENGTH / 2
            command.pos.y = self.SIDE_LENGTH / 2
            command.pos.z = self.ALTITUDE
            self._command_pub.publish(command)
            time.sleep(self.INTERVAL)

            # 頂点2
            command = PosVelYaw()
            command.pos.x = -self.SIDE_LENGTH / 2
            command.pos.y = self.SIDE_LENGTH / 2
            command.pos.z = self.ALTITUDE
            self._command_pub.publish(command)
            time.sleep(self.INTERVAL)

            # 頂点3
            command = PosVelYaw()
            command.pos.x = -self.SIDE_LENGTH / 2
            command.pos.y = -self.SIDE_LENGTH / 2
            command.pos.z = self.ALTITUDE
            self._command_pub.publish(command)
            time.sleep(self.INTERVAL)

            # 頂点4
            command = PosVelYaw()
            command.pos.x = self.SIDE_LENGTH / 2
            command.pos.y = -self.SIDE_LENGTH / 2
            command.pos.z = self.ALTITUDE
            self._command_pub.publish(command)
            time.sleep(self.INTERVAL)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandSquareTrajectoryNode()
    node.run()


if __name__ == "__main__":
    main()
