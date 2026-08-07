# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header

uint8 status
uint8 STATUS_OK = 0
uint8 STATUS_FRAME_LOST = 1
uint8 STATUS_TIMEOUT = 2
uint8 STATUS_OTHER = 3

float64 roll      # CH1: [-1, 1]
float64 pitch     # CH2: [-1, 1]
float64 throttle  # CH3: [-1, 1]
float64 yaw       # CH4: [-1, 1]
uint8 mode        # CH5: Flight Mode
bool sub_mode     # CH6: Sub Flight Mode
bool enable       # CH7: Enable Radio Control
bool kill         # CH8: Kill Switch
bool[8] gpsw      # CH9-16: General Purpose Switch
