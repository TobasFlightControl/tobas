#pragma once

#include <iostream>

#include "./ansi_text_styles.hpp"

/* Releaseビルドでも機能するアサーション． */
#define CHECK(expr)                                                                                                    \
  {                                                                                                                    \
    if (!static_cast<bool>(expr))                                                                                      \
    {                                                                                                                  \
      std::cout << RED_PREFIX << "Check failed: " << #expr << " (" << __FILE__ << ":" << __LINE__ << ")"               \
                << COLOR_RESET << std::endl;                                                                           \
      std::abort();                                                                                                    \
    }                                                                                                                  \
  }
