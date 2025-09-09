#pragma once

#include <string>

namespace ros2
{
/**
 * @brief 一時ファイルを生成する．
 *
 * @param path 作成されたファイルのパス
 * @return ファイルディスクリプタ
 */
int createTemporalFile(std::string& path);
}  // namespace ros2
