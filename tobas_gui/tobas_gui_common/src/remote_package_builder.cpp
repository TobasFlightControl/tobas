#include <tobas_constants/constants.hpp>

#include "../include/tobas_gui_common/remote_package_builder.hpp"
#include "../include/tobas_gui_common/constants.hpp"
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

  // NOTE: Paramikoは非対話型セッションを開始するため，コマンドごとに必要な環境変数を設定する必要がある．
  const auto ros2_setup_bash = (fs::path(kROS2JazzyPath) / "setup.bash").string();
  const auto tobas_setup_bash = (fs::path(kTobasPath) / "setup.bash").string();
  const auto pre_cmd = format(
    "source {} && "
    "source {} && "
    "cd {}",
    ros2_setup_bash, tobas_setup_bash, kColconWSPathRemote);

  // TODO: ビルド時間が長いため，PCでコンパイルしてから実行に必要なファイルのみを送る．
  const auto build_cmd = format(
    "colcon build "
    "--merge-install "
    "--symlink-install "
    "--parallel-workers $(nproc) "
    "--packages-up-to {}",
    meta_pkg_name);

  if (ssh_client_.execute(pre_cmd + " && " + build_cmd, output_, true) != ssh::SSHClient::E_NO_ERROR)
  {
    // ビルドできなければcleanして再試行
    RCLCPP_WARN(node_->get_logger(), "Failed to build. Retrying...");

    const auto command = pre_cmd + " && " + build_cmd + " --cmake-clean-first";
    if (ssh_client_.execute(command, output_, true) != ssh::SSHClient::E_NO_ERROR)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Clean build also failed: " << getErrorMessage());
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
