import os


def source_common_setup_bashes() -> int:
    return os.system("bash -c 'source /opt/ros/noetic/setup.bash && source /opt/tobas/setup.bash'")


def kill_gazebo() -> int:
    return os.system("killall -9 gzserver gzclient")
