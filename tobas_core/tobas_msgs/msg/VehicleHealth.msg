# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header

bool ok

uint8 realtime_compliance
uint8 battery_voltage
uint8 cpu_temperature
uint8 radio_link
uint8 rotor_links
uint8 attitude_level
uint8 position_stability
uint8 horizontal_position_accuracy
uint8 vertical_position_accuracy
uint8 attitude_accuracy
uint8 heading_accuracy
uint8 mag_offset
uint8 mag_alignment
uint8 vibration_level
uint8 user_defined_condition

uint8 PASSED = 0
uint8 FAILED = 1
uint8 IGNORED = 2
uint8 UNKNOWN = 3
