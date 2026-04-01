// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_yaml_tools/format.hpp"

#include <format>

#include <tobas_math/core.hpp>
#include <tobas_math/float.hpp>

namespace tobas
{
namespace yaml
{
/**
 * @brief 小数をyaml対応の文字列に変換する．
 *
 * e.g. 100 -> 100.0, 0.000012345 -> 1.2345e-05
 */
std::string format(double value, int prec)
{
  if (math::isInteger(value) && std::abs(static_cast<long>(value)) < math::ipow(10L, prec)) {
    // 小数部分を持たず有効数字ほどの桁数ではない場合は，整数表記に丸められてしまうため，小数部分を1桁追加する．
    return std::format("{:.1f}", value);
  }
  else {
    return std::format("{:.{}g}", value, prec);
  }
}
}  // namespace yaml
}  // namespace tobas
