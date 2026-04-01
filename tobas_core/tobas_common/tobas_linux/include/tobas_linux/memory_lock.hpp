// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstddef>

namespace tobas
{
namespace linux
{
/**
 * @brief Lock currently paged memory using mlockall.
 */
bool lockMemory();

/**
 * @brief Commit a pool of dynamic memory based on the memory already cached by this process
 * by checking the number of pagefaults.
 */
bool lockAndPrefaultDynamic();

/**
 * @brief Commit a pool of dynamic memory based on a prefixed size.
 */
bool lockAndPrefaultDynamic(size_t process_max_dynamic_memory);
}  // namespace linux
}  // namespace tobas
