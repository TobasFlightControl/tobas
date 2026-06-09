// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/error.hpp"

#include <cstring>

namespace tobas
{
namespace linux
{
std::string strError(int error_number)
{
  return "[Errno " + std::to_string(error_number) + "] " + strerror(error_number);
}
}  // namespace linux
}  // namespace tobas
