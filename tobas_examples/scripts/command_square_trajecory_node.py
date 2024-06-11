#!/usr/bin/env python3

import rospy
import actionlib

from tobas_msgs.msg import TakeoffAction, TakeoffGoal, PosVelAccYaw

ALTITUDE = 3.0  # [m]
SIDE_LENGTH = 5.0  # [m]
INTERVAL = 5.0  # [s]


if __name__ == "__main__":
    # ROSノードの初期化
    rospy.init_node("command_square_trajectory")

    # 離陸アクションクライアントの作成
    takeoff_client = actionlib.SimpleActionClient("takeoff_action", TakeoffAction)

    # アクションサーバーが起動するのを待つ
    takeoff_client.wait_for_server()

    # アクションゴールを作成
    takeoff_goal = TakeoffGoal()
    takeoff_goal.target_altitude = ALTITUDE
    takeoff_goal.duration = INTERVAL

    # アクションゴールを送信
    takeoff_client.send_goal_and_wait(takeoff_goal)

    # アクションの結果を取得
    if takeoff_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
        rospy.logerr("Takeoff action failed.")
        rospy.signal_shutdown()

    # コマンドのパブリッシャーを作成
    command_pub = rospy.Publisher("command/pos_vel_acc_yaw", PosVelAccYaw, queue_size=1)

    # 正方形の頂点を指令し続ける
    while not rospy.is_shutdown():
        # 頂点1
        command = PosVelAccYaw()
        command.pos.x = SIDE_LENGTH / 2
        command.pos.y = SIDE_LENGTH / 2
        command.pos.z = ALTITUDE
        command_pub.publish(command)
        rospy.sleep(INTERVAL)

        # 頂点2
        command = PosVelAccYaw()
        command.pos.x = -SIDE_LENGTH / 2
        command.pos.y = SIDE_LENGTH / 2
        command.pos.z = ALTITUDE
        command_pub.publish(command)
        rospy.sleep(INTERVAL)

        # 頂点3
        command = PosVelAccYaw()
        command.pos.x = -SIDE_LENGTH / 2
        command.pos.y = -SIDE_LENGTH / 2
        command.pos.z = ALTITUDE
        command_pub.publish(command)
        rospy.sleep(INTERVAL)

        # 頂点4
        command = PosVelAccYaw()
        command.pos.x = SIDE_LENGTH / 2
        command.pos.y = -SIDE_LENGTH / 2
        command.pos.z = ALTITUDE
        command_pub.publish(command)
        rospy.sleep(INTERVAL)
