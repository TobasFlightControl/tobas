// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>
#include <cstddef>

namespace tobas
{
namespace str
{
/* Concatenate `char` and `char*`. */
template <size_t N>
inline constexpr auto concat(char c, const char (&s)[N])
{
  std::array<char, N + 1> out{};  // `N` includes the terminating '\0'.
  out[0] = c;
  for (size_t i = 0; i < N; ++i) {
    out[i + 1] = s[i];
  }
  return out;  // Returning `data()` would release `std::array` and leave a dangling pointer.
}
}  // namespace str
}  // namespace tobas
