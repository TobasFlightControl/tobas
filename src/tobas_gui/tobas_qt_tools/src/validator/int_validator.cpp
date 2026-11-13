// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/validator/int_validator.hpp"

namespace tobas::qt
{
QValidator::State IntValidator::validate(QString& _input, int& _pos) const
{
  const auto original_res = super::validate(_input, _pos);

  if (original_res == Intermediate) {
    const auto extracted = locale().toInt(_input);
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
}  // namespace tobas::qt
