import rclpy
from rclpy import Node
import subprocess


class SimVehicleLauncher(Node):
    def __init__(self) -> None:
        super().__init__("sim_vehicle_launcher")

        self._timer = self.create_timer(0, self._run_sim_vehicle)

    def _run_sim_vehicle(self, event) -> None:
        # ArduCopterのシミュレータを起動
        # フレームタイプ (-fオプション) の頭に"gazebo-"とつく場合はGazeboインターフェースが起動する模様
        # Iris固有の設定は"ardupilot/Tools/autotest/default_params/gazebo-iris.parm"に書いてある
        try:
            subprocess.run(
                ". ~/.profile && sim_vehicle.py -v ArduCopter -f gazebo-iris -d 0 -w --ekf-single",
                shell=True,
                check=True,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to launch ArduPilot SITL: {e}")
            rclpy.shutdown()

        self._timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimVehicleLauncher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
