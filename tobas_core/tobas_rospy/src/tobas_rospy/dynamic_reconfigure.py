import rospy
import rosgraph
from typing import List


def find_dynamic_reconfigure_servers():
    """dynamic_reconfigureのサーバをリストアップ．"""
    # ROSマスターに接続
    master = rosgraph.Master(rospy.get_name())

    # すべてのサービス名を取得
    services: List[str, List[str]] = master.getSystemState()[2]

    # 動的再設定サービスを持つノードを探す
    dynamic_reconfigure_servers = []
    for service_name, service_servers in services:
        if service_name.endswith("set_parameters"):
            dynamic_reconfigure_servers.append(service_servers[0])

    return dynamic_reconfigure_servers
