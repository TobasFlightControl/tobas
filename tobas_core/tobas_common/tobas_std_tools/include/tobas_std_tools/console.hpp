// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <iostream>

#include "./ansi_text_styles.hpp"

/* Standard console output. */
#define PRINT_INFO(msg) std::cout << "[INFO] " << msg << std::endl

/* Print a positive status, such as ready. */
#define PRINT_GOOD(msg) std::cout << GREEN_PREFIX << "[INFO] " << msg << COLOR_RESET << std::endl

/* Print a warning message. */
#define PRINT_WARN(msg) std::cout << YELLOW_PREFIX << "[WARN] " << msg << COLOR_RESET << std::endl

/* Print an error message. */
#define PRINT_ERROR(msg) std::cerr << RED_PREFIX << "[ERROR] " << msg << COLOR_RESET << std::endl

#ifdef NDEBUG
/* Print a message only in debug mode. */
#define PRINT_DEBUG(msg) void(0)

/* Print a message only once and only in debug mode. */
#define PRINT_DEBUG_ONCE(msg) void(0)

#else
/* Print a message only in debug mode. */
#define PRINT_DEBUG(msg) std::cout << CYAN_PREFIX << "[DEBUG] " << msg << COLOR_RESET << std::endl

/* Print a message only once and only in debug mode. */
#define PRINT_DEBUG_ONCE(msg)                                                                                          \
  do {                                                                                                                 \
    static bool is_message_printed = false;                                                                            \
    if (!is_message_printed) {                                                                                         \
      PRINT_DEBUG(msg);                                                                                                \
      is_message_printed = true;                                                                                       \
    }                                                                                                                  \
  } while (false)
#endif
