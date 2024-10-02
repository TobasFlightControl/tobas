#pragma once

#include <cstddef>

namespace linux
{
/**
 * @brief Lock currently paged memory using mlockall.
 *
 * @param process_max_dynamic_memory
 * @return int Error code to propagate to main
 */
int lockMemory();

/**
 * @brief Commit a pool of dynamic memory based on the memory already cached by this process
 * by checking the number of pagefaults.
 *
 * @param process_max_dynamic_memory
 * @return int Error code to propagate to main
 */
int lockAndPrefaultDynamic();

/**
 * @brief Commit a pool of dynamic memory based on a prefixed size.
 *
 * @param process_max_dynamic_memory
 * @return int Error code to propagate to main
 */
int lockAndPrefaultDynamic(size_t process_max_dynamic_memory);
}  // namespace linux
