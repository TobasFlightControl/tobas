// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <iostream>

std::ostream& operator<<(std::ostream& _os, const tm& _arg);

double operator-(tm _lhs, tm _rhs);

template <typename Rep, typename Period>
std::ostream& operator<<(std::ostream& _os, const std::chrono::duration<Rep, Period>& _d)
{
  _os << _d.count() << " ";
  if constexpr (std::is_same_v<Period, std::ratio<1>>) {
    _os << "s";
  }
  else if constexpr (std::is_same_v<Period, std::milli>) {
    _os << "ms";
  }
  else if constexpr (std::is_same_v<Period, std::micro>) {
    _os << "μs";
  }
  else if constexpr (std::is_same_v<Period, std::nano>) {
    _os << "ns";
  }
  return _os;
}
