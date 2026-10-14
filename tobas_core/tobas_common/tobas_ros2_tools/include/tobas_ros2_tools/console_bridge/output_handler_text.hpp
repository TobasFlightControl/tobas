// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <console_bridge/console.h>

namespace console_bridge
{
/** Store console output as a string. */
class OutputHandlerText : public OutputHandler
{
public:
  explicit OutputHandlerText(LogLevel _level);

  void log(const std::string& _text, LogLevel _level, const char* _filename, int _line) override;

  const std::string& message() const;

  void clear();

private:
  const LogLevel level_;
  std::string msg_;
};
}  // namespace console_bridge
