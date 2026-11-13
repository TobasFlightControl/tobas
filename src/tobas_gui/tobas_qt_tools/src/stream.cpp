// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/stream.hpp"

std::ostream& operator<<(std::ostream& _os, const QString& _arg)
{
  _os << _arg.toStdString();
  return _os;
}
