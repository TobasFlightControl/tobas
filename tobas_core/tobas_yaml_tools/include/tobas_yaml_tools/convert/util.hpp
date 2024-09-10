#pragma once

#include <format>

namespace YAML
{
namespace util
{
/* 少数が自動的に整数に丸められるのを防ぐため，明示的に指数表示でエンコードする． */
inline std::string format(double value)
{
  // std::to_stringだと微小値が丸められてしまうためstd::formatを採用
  return std::format("{:e}", value);
}
}  // namespace util
}  // namespace YAML
