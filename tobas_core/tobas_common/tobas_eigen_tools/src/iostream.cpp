// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_eigen_tools/iostream.hpp"

std::ostream& operator<<(std::ostream& os, const Eigen::Quaterniond& arg)
{
  os << "x: " << arg.x() << ", y: " << arg.y() << ", z: " << arg.z() << ", w: " << arg.w();
  return os;
}
