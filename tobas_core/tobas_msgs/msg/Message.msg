# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header
uint8 level
string name
string message

# Message Level
uint8 LEVEL_DEBUG = 0
uint8 LEVEL_INFO = 1
uint8 LEVEL_WARN = 2
uint8 LEVEL_ERROR = 3
uint8 LEVEL_FATAL = 4
