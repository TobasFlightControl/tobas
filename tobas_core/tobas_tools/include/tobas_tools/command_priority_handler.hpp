#pragma once

#include <rclcpp/rclcpp.hpp>

namespace tobas
{
class CommandPriorityHandler
{
  static constexpr auto kHighestLevelTimeout = std::chrono::milliseconds(500);

public:
  explicit CommandPriorityHandler();

  /**
   * @brief コマンド優先度を受け取り，採用するか否かの判定を行う．
   *
   * コマンドを採用する条件:
   * 1. 現在の優先度以上の優先度
   * 2. 最後に最高優先度のコマンドを受けっとてから一定時間経過
   *
   * @param new_priority 受け取ったコマンド優先度．
   * @param cur_time 現在の時刻．
   * @return true コマンドを採用する場合．
   * @return false コマンドを採用しない場合．
   */
  bool update(const uint8_t& new_priority, const rclcpp::Time& cur_time);

private:
  uint8_t cur_priority_ = 0;
  rclcpp::Time t_last_highest_priority_;  // 最高優先度のコマンドが来た最後の時刻
};
}  // namespace tobas
