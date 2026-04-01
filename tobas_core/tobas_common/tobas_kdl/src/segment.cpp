// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/segment.hpp"

using namespace std;

namespace tobas
{
namespace kdl
{
bool Segment::isValid(string& error_msg) const
{
  if (name_.empty()) {
    error_msg = "Segment name is empty.";
    return false;
  }

  if (!joint_.isValid(error_msg)) {
    error_msg = name_ + "'s joint is invalid: " + error_msg;
    return false;
  }

  if (!f_tip_.isValid(error_msg)) {
    error_msg = name_ + "'s frame is invalid: " + error_msg;
    return false;
  }

  if (!I_.isValid(error_msg)) {
    error_msg = name_ + "'s inertia is invalid: " + error_msg;
    return false;
  }

  return true;
}

ostream& operator<<(ostream& os, const Segment& arg)
{
  os << "Name: " << arg.name_ << endl;
  os << "Joint:\n" << arg.joint_ << endl;
  os << "Frame:\n" << arg.f_tip_ << endl;
  os << "Inertia:\n" << arg.I_;
  return os;
}
}  // namespace kdl
}  // namespace tobas
