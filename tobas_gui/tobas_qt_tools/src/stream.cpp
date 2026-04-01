// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/stream.hpp"

std::ostream& operator<<(std::ostream& os, const QString& arg)
{
  os << arg.toStdString();
  return os;
}
