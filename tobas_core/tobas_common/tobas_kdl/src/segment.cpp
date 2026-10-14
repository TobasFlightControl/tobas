// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/segment.hpp"

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

std::ostream& operator<<(std::ostream& _os, const Segment& _arg)
{
  _os << "Name: " << _arg.name_ << '\n';
  _os << "Joint:\n" << _arg.joint_ << '\n';
  _os << "Frame:\n" << _arg.f_tip_ << '\n';
  _os << "Inertia:\n" << _arg.I_;
  return _os;
}
}  // namespace kdl
}  // namespace tobas
