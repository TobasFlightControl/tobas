// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/console_bridge/output_handler_text.hpp"

namespace console_bridge
{
OutputHandlerText::OutputHandlerText(LogLevel _level) : level_(_level)
{
}

void OutputHandlerText::log(const std::string& _text, LogLevel _level, const char*, int)
{
  if (_level >= level_) {
    msg_ += _text;
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
