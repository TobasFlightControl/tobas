// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace uadf
{
struct Thrust
{
  enum Direction
  {
    CW,
    CCW,
  } direction = CW;
};
}  // namespace uadf
}  // namespace tobas
