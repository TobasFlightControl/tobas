#!/usr/bin/env python3

import rospy
import subprocess
import signal

from tobas_rospy.utils import init_node


class SimVehicleLauncher:
    def __init__(self) -> None:
        self._timer = rospy.Timer(rospy.Duration(1e-3), self._run_sim_vehicle, oneshot=True)

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
            reason = "Failed to launch ArduPilot SITL"
            rospy.logerr(f"{reason}: {e}")
            rospy.signal_shutdown(reason)


if __name__ == "__main__":
    init_node()
    node = SimVehicleLauncher()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rospy.spin()
