// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <iostream>

#include <geometry_msgs/msg/quaternion.hpp>

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Quaternion& q);
