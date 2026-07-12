// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace linux
{
/* Return an error string in the form "[Errno errno] strerror(errno)". */
std::string strError(int error_number = errno);
}  // namespace linux
}  // namespace tobas
