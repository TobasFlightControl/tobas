// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>
#include <vector>

namespace tobas
{
namespace gui
{
namespace cmn
{
/**
 * @brief コマンドライン引数を編集する．
 *
 * cf. [rviz2/src/main.cpp](https://github.com/ros2/rviz/blob/rolling/rviz2/src/main.cpp)
 */
class NonRosArgumentParser
{
public:
  explicit NonRosArgumentParser(int argc, char** argv);

  int& argc();
  char** argv();

  /**
   * @brief ディスプレイサーバにX11を使うよう設定する．
   */
  bool setPlatformXcb();

private:
  std::vector<std::string> args_;

  int argc_;                 // argcのメモリ確保が必要
  std::vector<char*> argv_;  // argvのメモリ確保が必要
};
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
