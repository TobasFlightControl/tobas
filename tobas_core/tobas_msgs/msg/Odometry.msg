std_msgs/Header header

tobas_kdl_msgs/Frame frame  # The transformation from the global frame to the vehicle frame
tobas_kdl_msgs/Twist twist  # The 6D twist expressed in the body frame
tobas_kdl_msgs/Accel accel  # The 6D accel expressed in the body frame

tobas_eigen_msgs/Matrix3d position_covariance     # [m^2]
tobas_eigen_msgs/Matrix3d orientation_covariance  # [rad^2]
tobas_eigen_msgs/Matrix3d velocity_covariance     # [m^2/s^2]
tobas_eigen_msgs/Matrix3d gyro_covariance         # [rad^2/s^2]
