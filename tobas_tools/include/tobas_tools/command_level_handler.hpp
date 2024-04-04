#pragma once

#include <ros/ros.h>

namespace tobas
{
class CommandLevelHandler
{
  static constexpr double kHighestLevelTimeout = 0.5;

public:
  explicit CommandLevelHandler();

  /**
   * @brief コマンドレベルを受け取り，採用するか否かの判定を行う．
   *
   * コマンドを採用する条件:
   * 1. 現在のレベル以上のレベル
   * 2. 最後に最高レベルのコマンドを受けっとてから一定時間経過
   *
   * @param new_level 受け取ったコマンドレベル．
   * @param cur_time 現在の時刻．
   * @return true コマンドを採用する場合．
   * @return false コマンドを採用しない場合．
   */
  bool update(const uint8_t& new_level, const ros::Time& cur_time);

private:
  uint8_t cur_level_ = 0;
  ros::Time t_last_highest_level_;  // 最高レベルのコマンドが来た最後の時刻
};
}  // namespace tobas
