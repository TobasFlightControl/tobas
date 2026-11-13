# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

string link_name

float64 speed   # [rad/s]
float64 thrust  # [N]

uint8 status
uint8 NO_ERROR = 0
uint8 COMMUNICATION_FAILURE = 1
