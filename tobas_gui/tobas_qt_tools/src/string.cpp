// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/string.hpp"

namespace tobas
{
namespace qt
{
QString boolToText(bool _arg)
{
  if (_arg) {
    return "true";
  }
  else {
    return "false";
  }
}

QStringList stringListFromStdToQt(const std::vector<std::string>& _src)
{
  QStringList res;
  for (const auto& item : _src) {
    res.append(QString::fromStdString(item));
  }
  return res;
}

std::vector<std::string> stringListFromQtToStd(const QStringList& _src)
{
  std::vector<std::string> res;
  for (const auto& item : _src) {
    res.push_back(item.toStdString());
  }
  return res;
}

bool isControlChar(const QChar& _c)
{
  const auto cat = _c.category();
  return cat == QChar::Other_Control || cat == QChar::Other_Format;
}

bool containsControlChars(const QStringView& _s)
{
  for (const auto& c : _s) {
    if (isControlChar(c)) {
      return true;
    }
  }
  return false;
}
}  // namespace qt
}  // namespace tobas
