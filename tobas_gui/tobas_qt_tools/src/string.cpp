// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/string.hpp"

#include <QRegularExpression>

namespace tobas
{
namespace qt
{
QString boolToText(bool arg)
{
  if (arg) {
    return "true";
  }
  else {
    return "false";
  }
}

QStringList stringListFromStdToQt(const std::vector<std::string>& src)
{
  QStringList res;
  for (const auto& item : src) {
    res.append(QString::fromStdString(item));
  }
  return res;
}

std::vector<std::string> stringListFromQtToStd(const QStringList& src)
{
  std::vector<std::string> res;
  for (const auto& item : src) {
    res.push_back(item.toStdString());
  }
  return res;
}

bool isControlChar(const QChar& c)
{
  const auto cat = c.category();
  return cat == QChar::Other_Control || cat == QChar::Other_Format;
}

bool containsControlChars(const QStringView& s)
{
  for (const auto& c : s) {
    if (isControlChar(c)) {
      return true;
    }
  }
  return false;
}
}  // namespace qt
}  // namespace tobas
