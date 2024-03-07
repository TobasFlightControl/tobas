#!/usr/bin/env python3

import rospy
import os.path as osp
import subprocess
import signal


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
    node_name = osp.splitext(osp.basename(__file__))[0]
    rospy.init_node(node_name)
    node = SimVehicleLauncher()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rospy.spin()
