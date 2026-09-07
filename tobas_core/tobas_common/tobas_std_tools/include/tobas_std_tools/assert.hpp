// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <iostream>

#include "./ansi_text_styles.hpp"

#ifdef NDEBUG
#define assertWithMsg(expr, msg)                                                                                       \
  {                                                                                                                    \
    void(0); /* Do nothing. */                                                                                         \
  }
#else
#define assertWithMsg(expr, msg)                                                                                       \
  {                                                                                                                    \
    if (!static_cast<bool>(expr)) /* The condition `expr` is evaluated for the first time here. */                     \
    {                                                                                                                  \
      std::cout << ::tobas::st::kRedPrefix << __FILE__ << ":" << __LINE__ << ": Assertion failed: " << msg             \
                << ::tobas::st::kColorReset << std::endl;                                                              \
      std::abort();                                                                                                    \
    }                                                                                                                  \
  }
#endif
