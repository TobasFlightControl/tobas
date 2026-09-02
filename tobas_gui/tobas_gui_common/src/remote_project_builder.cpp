// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/remote_project_builder.hpp"

#include <QDebug>

#include <tobas_constants/path.hpp>

#include "tobas_gui_common/project_paths.hpp"

namespace tobas
{
namespace gui
{
namespace cmn
{
RemoteProjectBuilder::RemoteProjectBuilder(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

bool RemoteProjectBuilder::build(const QString& remote_proj_path)
{
  // Paramiko starts non-interactive sessions, so required environment variables must be set for each command.
  const auto pre_cmd =
    QString("source /opt/ros/jazzy/setup.bash && source /opt/tobas/local_setup.bash && cd %1").arg(kColconWSPathRoot);

  // `--symlink-install` does not work with root privileges.
  const auto meta_pkg_name = cmn::ProjectPaths(remote_proj_path).metaPkgName();
  const auto build_cmd =
    QString("colcon build "
            "--merge-install "
            "--parallel-workers $(nproc) "
            "--cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS='-mcpu=native' -DCMAKE_CXX_FLAGS='-mcpu=native' "
            "--packages-up-to %1")
      .arg(meta_pkg_name);

  // Finish if the build succeeds.
  const auto error = ssh_client_.execute(pre_cmd + " && " + build_cmd, output_, true);
  if (error != ssh::SshClient::kNoError) {
    qWarning() << "Failed to build the remote package.";
    return false;
  }

  return true;
}

const QString& RemoteProjectBuilder::getOutput() const
{
  return output_;
}

QString RemoteProjectBuilder::getErrorMessage() const
{
  return ssh_client_.errorMessage();
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
