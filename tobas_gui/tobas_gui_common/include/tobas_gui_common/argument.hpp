#pragma once

#include <string>
#include <vector>

namespace gui
{
namespace common
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
}  // namespace common
}  // namespace gui
