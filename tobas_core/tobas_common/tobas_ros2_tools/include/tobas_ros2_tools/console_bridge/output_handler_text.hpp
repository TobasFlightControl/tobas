#pragma once

#include <console_bridge/console.h>

namespace console_bridge
{
/* コンソール出力を文字列として保持する． */
class OutputHandlerText : public OutputHandler
{
public:
  explicit OutputHandlerText(LogLevel level);

  void log(const std::string& text, LogLevel level, const char* filename, int line) override;

  const std::string& message() const;

  void clear();

private:
  const LogLevel level_;
  std::string msg_;
};
}  // namespace console_bridge
