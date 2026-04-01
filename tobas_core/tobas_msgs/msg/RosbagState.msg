# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

std_msgs/Header header
bool recording
string file_path
builtin_interfaces/Duration duration
uint64 file_size  # [byte]
uint64 message_count
