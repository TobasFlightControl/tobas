// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>
#include <vector>

#include <QStringList>

namespace tobas
{
namespace qt
{
QStringList stringListFromStdToQt(const std::vector<std::string>& src);
std::vector<std::string> stringListFromQtToStd(const QStringList& src);

bool isControlChar(const QChar& c);

bool containsControlChars(const QStringView& s);
}  // namespace qt
}  // namespace tobas
