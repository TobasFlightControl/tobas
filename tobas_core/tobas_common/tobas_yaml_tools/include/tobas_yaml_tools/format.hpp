#pragma once

#include <string>

namespace yaml
{
/**
 * @brief 小数をyaml対応の文字列に変換する．
 *
 * e.g. 100 -> 100.0, 0.000012345 -> 1.2345e-05
 */
std::string format(double value, int prec = 9);
}  // namespace yaml
