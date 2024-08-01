from tobas_rclpy.utils import init_node
from tobas_multirotor_trajectory import FollowTrajectoryClient_ForwardBackward

if __name__ == "__main__":
    init_node()
    node = FollowTrajectoryClient_ForwardBackward()
    node.run()
