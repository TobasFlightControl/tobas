// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace yaml
{
/**
 * @brief 小数をyaml対応の文字列に変換する．
 *
 * e.g. 100 -> 100.0, 0.000012345 -> 1.2345e-05
 */
std::string format(double value, int prec = 9);
}  // namespace yaml
}  // namespace tobas
