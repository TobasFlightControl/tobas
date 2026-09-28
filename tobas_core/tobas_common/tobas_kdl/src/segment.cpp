// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/segment.hpp"

using namespace std;

namespace tobas
{
namespace kdl
{
std::expected<void, std::string> Segment::validate() const
{
  if (name_.empty()) {
    return std::unexpected("Segment name is empty.");
  }

  if (const auto result = joint_.validate(); !result) {
    return std::unexpected(name_ + "'s joint is invalid: " + result.error());
  }

  if (const auto result = f_tip_.validate(); !result) {
    return std::unexpected(name_ + "'s frame is invalid: " + result.error());
  }

  if (const auto result = I_.validate(); !result) {
    return std::unexpected(name_ + "'s inertia is invalid: " + result.error());
  }

  return {};
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
