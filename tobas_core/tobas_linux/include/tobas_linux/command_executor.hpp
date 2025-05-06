#pragma once

#include <array>
#include <string>

namespace linux
{
class CommandExecutor
{
public:
  explicit CommandExecutor();

  bool execute(const std::string& command);

  inline const std::string& getOutput() const;

private:
  std::array<char, 128> buffer_;
  std::string output_;
};

inline const std::string& CommandExecutor::getOutput() const
{
  return output_;
}
}  // namespace linux
