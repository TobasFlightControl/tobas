// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_eigen_tools/iostream.hpp"

std::ostream& operator<<(std::ostream& _os, const Eigen::Quaterniond& _arg)
{
  _os << "x: " << _arg.x() << ", y: " << _arg.y() << ", z: " << _arg.z() << ", w: " << _arg.w();
  return _os;
}
