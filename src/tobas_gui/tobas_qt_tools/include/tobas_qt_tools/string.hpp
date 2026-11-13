// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>
#include <vector>

#include <QStringList>

namespace tobas::qt
{
/** boolean -> "true" or "false" */
QString boolToText(bool _arg);

QStringList stringListFromStdToQt(const std::vector<std::string>& _src);
std::vector<std::string> stringListFromQtToStd(const QStringList& _src);

bool isControlChar(const QChar& _c);

bool containsControlChars(const QStringView& _s);
}  // namespace tobas::qt
