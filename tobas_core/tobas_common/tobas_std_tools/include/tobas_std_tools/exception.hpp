// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <exception>
#include <string>

namespace tobas
{
namespace st
{
class NotImplementedError : public std::exception
{
public:
  NotImplementedError(const std::string& msg = "");

  virtual const char* what() const noexcept;

private:
  std::string msg_;
};
}  // namespace st
}  // namespace tobas
