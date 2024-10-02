#pragma once

#include <cstdint>
#include <cstddef>

namespace linux
{
class ProcessSettings
{
  static constexpr char kOptionLockMemory[] = "--lock-memory";
  static constexpr char kOptionLockMemorySize[] = "--lock-memory-size";
  static constexpr char kOptionPriority[] = "--priority";
  static constexpr char kOptionCPUAffinity[] = "--cpu-affinity";

public:
  bool init(int argc, char* argv[]);

  bool configureProcess();

  void printUsage();

private:
  bool lock_memory_ = false;        // Lock and prefault memory
  int process_priority_ = 0;        // Process priority value to set
  uint32_t cpu_affinity_ = 0;       // Process cpu affinity value to set
  size_t lock_memory_size_mb_ = 0;  // Memory size to lock in Megabytes
};
}  // namespace linux
