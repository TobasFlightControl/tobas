from tobas_rclpy.utils import init_node
from tobas_multirotor_trajectory import FollowTrajectoryClient_Hover

if __name__ == "__main__":
    init_node()
    node = FollowTrajectoryClient_Hover()
    node.run()
