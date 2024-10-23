#pragma once

#include <cstdio>
#include <string>

namespace YAML
{
namespace util
{
/* 少数が自動的に整数に丸められるのを防ぐため，明示的に指数表示でエンコードする． */
inline std::string format(double value)
{
  // std::to_stringだと微小値が丸められてしまう
  // std::formatはGCC-13以上でないと使えない
  char buffer[15];  // snprintfは最大長さ15の文字列を返す
  std::snprintf(buffer, sizeof(buffer), "%e", value);
  return std::string(buffer);  // char*のままだとメモリ開放されてしまうためstd::stringに変換
}
}  // namespace util
}  // namespace YAML
