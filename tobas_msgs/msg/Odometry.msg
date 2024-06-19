std_msgs/Header header

tobas_kdl_msgs/Frame frame
tobas_kdl_msgs/Twist twist
tobas_kdl_msgs/Accel accel

tobas_eigen_msgs/Matrix3d position_covariance              # [m^2]
tobas_eigen_msgs/Matrix3d orientation_covariance           # [rad^2]
tobas_eigen_msgs/Matrix3d linear_velocity_covariance       # [m^2/s^2]
tobas_eigen_msgs/Matrix3d angular_velocity_covariance      # [rad^2/s^2]
tobas_eigen_msgs/Matrix3d linear_acceleration_covariance   # [m^2/s^2/4]
tobas_eigen_msgs/Matrix3d angular_acceleration_covariance  # [rad^2/s^4]

int8 status
int8 NO_ERROR = 0
int8 POSITION_LOST = -1
