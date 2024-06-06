import os
import os.path as osp
import rospy
import inspect
from urllib.parse import urlparse
from argparse import ArgumentParser


def get_ros_master_ip() -> str:
    ros_master_uri = os.environ.get("ROS_MASTER_URI")
    return urlparse(ros_master_uri).hostname


def get_log_level() -> int:
    """コマンドライン引数からログレベルを取得する．"""
    parser = ArgumentParser()
    parser.add_argument("--log_level", type=str, choices=["debug", "info", "warn", "error", "fatal"], default="info")
    args, _ = parser.parse_known_args()
    return getattr(rospy, args.log_level.upper())


def init_node() -> None:
    """ノードを起動する．"""
    # ログレベルを取得
    log_level = get_log_level()

    # 実行スクリプトからノード名を取得
    stack = inspect.stack()
    node_file = stack[-1].filename  # 実行スクリプトのパス
    node_name = osp.splitext(osp.basename(node_file))[0]

    # ノード名から接尾語を削除
    suffix = "_node"
    if node_name.endswith(suffix):
        node_name = node_name[: -len(suffix)]

    # ノードを起動
    rospy.init_node(node_name, log_level=log_level)
