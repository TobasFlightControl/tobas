// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>
#include <cstddef>

namespace tobas
{
namespace str
{
/* char と char* を接続する． */
template <size_t N>
inline constexpr auto concat(char c, const char (&s)[N])
{
  std::array<char, N + 1> out{};  // N は終端 '\0' を含む
  out[0] = c;
  for (size_t i = 0; i < N; ++i) {
    out[i + 1] = s[i];
  }
  return out;  // data() を返すと std::array が開放されてダングリングポインタになってしまうことに注意
}
}  // namespace str
}  // namespace tobas
