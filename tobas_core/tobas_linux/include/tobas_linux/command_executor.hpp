#pragma once

#include <string>
#include <array>

namespace linux
{
class CommandExecutor
{
public:
  explicit CommandExecutor();

  bool execute(const std::string& command);

  inline const std::string& getOutput() const
  {
    return output_;
  }

private:
  std::array<char, 128> buffer_;
  std::string output_;
};
}  // namespace linux
