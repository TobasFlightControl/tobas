// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>
#include <string>

namespace tobas
{
namespace linux
{
class CommandExecutor
{
public:
  explicit CommandExecutor();

  bool execute(std::string command);

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
}  // namespace tobas
