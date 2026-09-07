// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <iostream>

#include "./ansi_text_styles.hpp"

/* Assertion that also works in release builds. */
#define TOBAS_CHECK(expr)                                                                                              \
  {                                                                                                                    \
    if (!static_cast<bool>(expr)) {                                                                                    \
      std::cout << ::tobas::st::kRedPrefix << "Check failed: " << #expr << " (" << __FILE__ << ":" << __LINE__ << ")"  \
                << ::tobas::st::kColorReset << std::endl;                                                              \
      std::abort();                                                                                                    \
    }                                                                                                                  \
  }
