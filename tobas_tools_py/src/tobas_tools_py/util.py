from tobas_rospy.utils import get_ros_ip


def is_running_under_fc_master() -> bool:
    """自身のノードがFC側のROSマスターに属する場合にTrueを返す．"""
    return get_ros_ip().startswith("192.168.249.")
