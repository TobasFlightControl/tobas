// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <iostream>

#include <eigen3/Eigen/Geometry>

std::ostream& operator<<(std::ostream& _os, const Eigen::Quaterniond& _arg);
