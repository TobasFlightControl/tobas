// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/remote_project_builder.hpp"

#include <tobas_constants/path.hpp>

#include "tobas_gui_common/project_paths.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace cmn
{
RemoteProjectBuilder::RemoteProjectBuilder(rclcpp::Node::SharedPtr node) : node_(node), ssh_client_(node)
{
}

bool RemoteProjectBuilder::build(const fs::path& remote_proj_path)
{
  const auto meta_pkg_name = cmn::ProjectPaths(remote_proj_path).metaPkgName();

  // Paramikoは非対話型セッションを開始するため，コマンドごとに必要な環境変数を設定する必要がある．
  const auto ros2_setup_bash = (fs::path(kRos2JazzyInstallPath) / "setup.bash").string();
  const auto tobas_setup_bash = (fs::path(kTobasInstallPath) / "local_setup.bash").string();
  const auto pre_cmd = std::format(
    "source {} && "
    "source {} && "
    "cd {}",
    ros2_setup_bash,
    tobas_setup_bash,
    kColconWSPathRoot);

  // ルート権限だと--symlink-installが機能しない
  const auto build_cmd = std::format(
    "colcon build "
    "--merge-install "
    "--parallel-workers $(nproc) "
    "--cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS=\"-mcpu=native\" -DCMAKE_CXX_FLAGS=\"-mcpu=native\" "
    "--packages-up-to {}",
    meta_pkg_name);

  // ビルドできれば終了
  if (ssh_client_.execute(pre_cmd + " && " + build_cmd, output_, true) != ssh::SshClient::kNoError) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to build the remote package.");
    return false;
  }

  return true;
}

const std::string& RemoteProjectBuilder::getOutput() const
{
  return output_;
}

const char* RemoteProjectBuilder::getErrorMessage() const
{
  return ssh_client_.errorMessage();
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
