// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <iostream>

std::ostream& operator<<(std::ostream& os, const tm& arg);

double operator-(tm lhs, tm rhs);

template <typename Rep, typename Period>
std::ostream& operator<<(std::ostream& os, const std::chrono::duration<Rep, Period>& d)
{
  os << d.count() << " ";
  if constexpr (std::is_same_v<Period, std::ratio<1>>) {
    os << "s";
  }
  else if constexpr (std::is_same_v<Period, std::milli>) {
    os << "ms";
  }
  else if constexpr (std::is_same_v<Period, std::micro>) {
    os << "μs";
  }
  else if constexpr (std::is_same_v<Period, std::nano>) {
    os << "ns";
  }
  return os;
}
