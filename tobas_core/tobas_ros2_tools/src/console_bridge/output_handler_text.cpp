#include "tobas_ros2_tools/console_bridge/output_handler_text.hpp"

namespace console_bridge
{
OutputHandlerText::OutputHandlerText(LogLevel level) : level_(level)
{
}

void OutputHandlerText::log(const std::string& text, LogLevel level, const char*, int)
{
  if (level >= level_) {
    msg_ += text;
    msg_ += '\n';
  }
}

const std::string& OutputHandlerText::message() const
{
  return msg_;
}

void OutputHandlerText::clear()
{
  msg_.clear();
}
}  // namespace console_bridge
