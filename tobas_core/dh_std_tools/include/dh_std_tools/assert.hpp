#pragma once

#include <iostream>

#include "./ansi_text_styles.hpp"

#ifdef NDEBUG
#define assertWithMsg(expr, msg)                                                                   \
  {                                                                                                \
    void(0); /* 何もしない */                                                                 \
  }
#else
#define assertWithMsg(expr, msg)                                                                   \
  {                                                                                                \
    if (!static_cast<bool>(expr)) /* ここで初めて条件exprが評価される */             \
    {                                                                                              \
      std::cout << RED_PREFIX << __FILE__ << ":" << __LINE__ << ": Assertion failed: " << msg      \
                << COLOR_RESET << std::endl;                                                       \
      std::abort();                                                                                \
    }                                                                                              \
  }
#endif
