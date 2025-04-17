#include <tobas_constants/constants.hpp>

#include "../include/tobas_gui_common/remote_package_builder.hpp"
#include "../include/tobas_gui_common/package.hpp"

using namespace std;
namespace fs = filesystem;

namespace gui
{
namespace common
{
RemotePackageBuilder::RemotePackageBuilder(rclcpp::Node::SharedPtr node) : node_(node), ssh_client_(node)
{
}

bool RemotePackageBuilder::build(const fs::path& remote_tbs_path)
{
  const auto meta_pkg_name = common::getTBSMetaName(remote_tbs_path);

  // XXX: Paramikoは非対話型セッションを開始するため，コマンドごとに必要な環境変数を設定する必要がある．
  const auto ros2_setup_bash = (fs::path(tobas::kROS2JazzyInstallPath) / "setup.bash").string();
  const auto tobas_setup_bash = (fs::path(tobas::kTobasInstallPath) / "setup.bash").string();
  const auto pre_cmd = format(
    "source {} && "
    "source {} && "
    "cd {}",
    ros2_setup_bash, tobas_setup_bash, tobas::kColconWSPathRoot);

  // XXX: ルート権限だと--symlink-installが機能しない
  const auto build_cmd = format(
    "colcon build "
    "--merge-install "
    "--parallel-workers $(nproc) "
    "--cmake-args -DCMAKE_C_COMPILER=/usr/local/bin/gcc -DCMAKE_CXX_COMPILER=/usr/local/bin/g++ "
    "--packages-up-to {}",
    meta_pkg_name);

  if (ssh_client_.execute(pre_cmd + " && " + build_cmd, output_, true) != ssh::SSHClient::E_NO_ERROR)
  {
    // ビルドできなければcleanして再試行
    RCLCPP_WARN(node_->get_logger(), "Failed to build remote package. Retrying...");

    const auto command = pre_cmd + " && sudo colcon clean workspace -y && " + build_cmd;
    if (ssh_client_.execute(command, output_, true) != ssh::SSHClient::E_NO_ERROR)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Clean build of remote package also failed: " << getErrorMessage());
      return false;
    }
  }

  return true;
}

const string& RemotePackageBuilder::getOutput() const
{
  return output_;
}

const char* RemotePackageBuilder::getErrorMessage() const
{
  return ssh_client_.errorMessage();
}
}  // namespace common
}  // namespace gui
