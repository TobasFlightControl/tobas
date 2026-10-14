// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace dsp
{
template <typename T>
class BaseFilter
{
public:
  virtual void update(const T& _u, double _dt) = 0;

  virtual const T& getValue() const = 0;
  virtual void setValue(const T& _x) = 0;
};
}  // namespace dsp
}  // namespace tobas
