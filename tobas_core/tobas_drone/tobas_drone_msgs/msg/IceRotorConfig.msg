# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

tobas_drone_msgs/RotorConfig rotor

float64 gear_ratio
tobas_std_msgs/RangeFloat64 pitch_limit
float64 center_pitch
VppMotorConstant motor_const
VppMomentConstant moment_const
uint8 hw_iface
