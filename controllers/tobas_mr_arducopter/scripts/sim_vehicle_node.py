#!/usr/bin/env python3

import rospy
import os.path as osp
import subprocess


class SimVehicleLauncher:
    DEFAULT_FRAME_TYPE = "quad"

    def __init__(self) -> None:
        self._frame_type = rospy.get_param("~frame_type", self.DEFAULT_FRAME_TYPE)

        self._timer = rospy.Timer(rospy.Time(0), self._run_sim_vehicle, oneshot=True)

    def _run_sim_vehicle(self, event) -> None:
        subprocess.run(
            f"sim_vehicle.py -v ArduCopter -f {self._frame_type}",
            shell=True,
            check=True,
        )


if __name__ == "__main__":
    node_name = osp.splitext(osp.basename(__file__))[0]
    rospy.init_node(node_name)
    node = SimVehicleLauncher()
    rospy.spin()
