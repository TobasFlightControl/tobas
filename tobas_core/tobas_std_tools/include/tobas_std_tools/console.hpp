#pragma once

#include <iostream>

#include "./ansi_text_styles.hpp"

/* 標準のコンソール出力． */
#define PRINT_INFO(msg) std::cout << "[INFO] " << msg << std::endl

/* 準備完了などの何かしら「良い」状態を出力する． */
#define PRINT_GOOD(msg) std::cout << GREEN_PREFIX << "[INFO] " << msg << COLOR_RESET << std::endl

/* 警告文を出力する． */
#define PRINT_WARN(msg) std::cout << YELLOW_PREFIX << "[WARN] " << msg << COLOR_RESET << std::endl

/* エラー文を出力する． */
#define PRINT_ERROR(msg) std::cout << RED_PREFIX << "[ERROR] " << msg << COLOR_RESET << std::endl

#ifdef NDEBUG
/* デバッグモードのときに限りメッセージを出力する． */
#define PRINT_DEBUG(msg) void(0)

/* デバッグモードのときに限り一度だけメッセージを出力する． */
#define PRINT_DEBUG_ONCE(msg) void(0)

#else
/* デバッグモードのときに限りメッセージを出力する． */
#define PRINT_DEBUG(msg) std::cout << CYAN_PREFIX << "[DEBUG] " << msg << COLOR_RESET << std::endl

/* デバッグモードのときに限り一度だけメッセージを出力する． */
#define PRINT_DEBUG_ONCE(msg)                                                                      \
  do                                                                                               \
  {                                                                                                \
    static bool is_message_printed = false;                                                        \
    if (!is_message_printed)                                                                       \
    {                                                                                              \
      PRINT_DEBUG(msg);                                                                            \
      is_message_printed = true;                                                                   \
    }                                                                                              \
  } while (false)
#endif
