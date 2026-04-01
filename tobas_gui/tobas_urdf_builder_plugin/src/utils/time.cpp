// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/utils/time.hpp"

#include <chrono>

namespace ch = std::chrono;

namespace tobas
{
namespace gui
{
namespace ub
{
namespace utils
{
int timeNowMilliseconds()
{
  const auto now = ch::system_clock::now().time_since_epoch();
  const auto ms = ch::duration_cast<ch::milliseconds>(now);
  return static_cast<int>(ms.count());
}
}  // namespace utils
}  // namespace ub
}  // namespace gui
}  // namespace tobas
