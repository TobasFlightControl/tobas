// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <bit>
#include <iostream>

#include <tobas_algorithm/binary.hpp>

using namespace std;

int main()
{
  constexpr auto src = 1.2345f16;
  constexpr auto bits = bit_cast<uint16_t>(src);

  const auto res1 = bit_cast<float16_t>(bits);
  const auto res2 = tobas::algo::decodeR16(bits);

  if (res1 != res2) {
    cerr << "Failed to cast U16 to F16: " << res1 << " != " << res2 << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
