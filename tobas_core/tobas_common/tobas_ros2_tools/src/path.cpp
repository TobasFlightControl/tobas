// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/path.hpp"

#include <rcpputils/filesystem_helper.hpp>

namespace rfs = rcpputils::fs;

namespace tobas
{
namespace ros2
{
int createTemporalFile(std::string& path)
{
  // システムの一時ディレクトリを取得
  const auto tmp_dir = rfs::temp_directory_path();

  // テンプレート文字列を作成．末尾6文字がXでなければならない．
  path = (tmp_dir / "tobas_temporal_file_XXXXXX").string();

  // mkstemp()はテンプレートのX部分をランダムな文字列に置き換え，一時ファイルを作成する．
  return mkstemp(path.data());
}
}  // namespace ros2
}  // namespace tobas
