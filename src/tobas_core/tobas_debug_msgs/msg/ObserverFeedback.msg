# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header

tobas_eigen_msgs/Vector3d position
tobas_eigen_msgs/Vector3d velocity
tobas_eigen_msgs/Vector4d hamilton
tobas_eigen_msgs/Vector3d accel_bias
tobas_eigen_msgs/Vector3d gyro_bias
tobas_eigen_msgs/Vector3d mag_hard_bias
tobas_eigen_msgs/Matrix3d mag_soft_bias
float64 baro_alt_bias
float64 gravity

tobas_eigen_msgs/Matrix3d position_cov
tobas_eigen_msgs/Matrix3d velocity_cov
tobas_eigen_msgs/Matrix3d rotation_cov
tobas_eigen_msgs/Matrix3d accel_bias_cov
tobas_eigen_msgs/Matrix3d gyro_bias_cov
tobas_eigen_msgs/Matrix3d mag_hard_bias_cov
tobas_eigen_msgs/Matrix6d mag_soft_bias_cov
float64 baro_alt_bias_var
float64 gravity_var

float64 gnss_anomaly_score
