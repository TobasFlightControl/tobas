import os.path as osp
import time
import rclpy

from tobas_rqt_tools.roslaunch import create_launcher

WAIT_NODELET_MANAGER = 3


if __name__ == "__main__":
    node_name = osp.splitext(osp.basename(__file__))[0]
    rclpy.init_node(node_name)

    # Get rosparams
    nodelet = rclpy.get_param("~nodelet")

    if nodelet:
        # Launch nodelet_manager.launch
        create_launcher("{{ pkg_name }}", "nodelet_manager.launch")

        # Wait for nodelet manager
        time.sleep(WAIT_NODELET_MANAGER)

    # Launch hardware_interfaces.launch
    create_launcher("{{ pkg_name }}", "hardware_interfaces.launch", [f"nodelet:={nodelet}"])

    # Launch bringup.launch
    create_launcher("{{ pkg_name }}", "bringup.launch", [f"nodelet:={nodelet}"])

    rclpy.spin()
