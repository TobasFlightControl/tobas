from tobas_rospy.utils import get_ros_master_ip

from .constants import ROS_MASTER_URI_FC


def is_running_under_fc_master() -> bool:
    """自身のノードがFC側のROSマスターに属する場合にTrueを返す．"""
    return get_ros_master_ip() == ROS_MASTER_URI_FC
