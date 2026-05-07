# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

string name

tobas_drone_msgs/JointConfig[] joints
tobas_drone_msgs/PwmConfig[] pwms

int8 prop_type
tobas_drone_msgs/ElectricPropulsionSystemConfig eprop
tobas_drone_msgs/IcePropulsionSystemConfig iprop

bool has_fixed_wing
tobas_drone_msgs/FixedWingConfig fixed_wing

uint32 num_sbus_channels
