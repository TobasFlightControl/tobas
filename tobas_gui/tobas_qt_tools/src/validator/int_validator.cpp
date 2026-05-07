// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/validator/int_validator.hpp"

namespace tobas
{
namespace qt
{
QValidator::State IntValidator::validate(QString& input, int& pos) const
{
  const auto original_res = super::validate(input, pos);

  if (original_res == Intermediate) {
    const auto extracted = locale().toInt(input);
    if (extracted > 0) {
      if (extracted > top() && -extracted < bottom()) {
        return Invalid;
      }
    }
    else if (extracted < bottom()) {
      return Invalid;
    }
  }

  return original_res;
}
}  // namespace qt
}  // namespace tobas
