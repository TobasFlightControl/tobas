#include "tobas_gui_common/remote_project_builder.hpp"

#include <QEventLoop>

#include <tobas_constants/constants.hpp>

#include "tobas_gui_common/project_paths.hpp"

namespace fs = std::filesystem;

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
  const auto ros2_setup_bash = (fs::path(tobas::kRos2JazzyInstallPath) / "setup.bash").string();
  const auto tobas_setup_bash = (fs::path(tobas::kTobasInstallPath) / "local_setup.bash").string();
  const auto pre_cmd = std::format(
    "source {} && "
    "source {} && "
    "cd {}",
    ros2_setup_bash,
    tobas_setup_bash,
    tobas::kColconWSPathRoot);

  // ルート権限だと--symlink-installが機能しない
  const auto build_cmd = std::format(
    "colcon build "
    "--merge-install "
    "--parallel-workers $(nproc) "
    "--cmake-args -DCMAKE_C_COMPILER=/usr/local/bin/gcc -DCMAKE_CXX_COMPILER=/usr/local/bin/g++ "  // コンパイラを指定
    "--packages-up-to {}",
    meta_pkg_name);

  // ビルドできれば終了
  if (ssh_client_.execute(pre_cmd + " && " + build_cmd, output_, true) == ssh::SSHClient::kNoError) {
    return true;
  }

  // ビルドできなければクリーンビルド
  RCLCPP_WARN(node_->get_logger(), "Failed to build remote package. Retrying...");
  const auto command = pre_cmd + " && sudo colcon clean workspace -y && " + build_cmd;
  if (ssh_client_.execute(command, output_, true) == ssh::SSHClient::kNoError) {
    return true;
  }

  // クリーンビルドもできなければエラー
  RCLCPP_ERROR_STREAM(node_->get_logger(), "Clean build of remote package also failed: " << getErrorMessage());
  return false;
}

const std::string& RemoteProjectBuilder::getOutput() const
{
  return output_;
}

const char* RemoteProjectBuilder::getErrorMessage() const
{
  return ssh_client_.errorMessage();
}

RemoteProjectBuilderThread::RemoteProjectBuilderThread(
  rclcpp::Node::SharedPtr node,
  const std::filesystem::path& proj_path)
  : proj_path_(proj_path), builder_(node)
{
}

void RemoteProjectBuilderThread::run()
{
  if (!builder_.build(proj_path_)) {
    Q_EMIT finished(false, QString::fromStdString(builder_.getErrorMessage()));
    return;
  }

  Q_EMIT finished(true, "");
}

std::expected<void, QString>
buildRemoteProjectBackground(rclcpp::Node::SharedPtr node, const std::filesystem::path& proj_path)
{
  // スレッドを作成
  cmn::RemoteProjectBuilderThread thread(node, proj_path);

  // 別スレッドの結果をキャッチするためのイベントループを用意
  bool success;
  QString message;
  QEventLoop loop;
  QObject::connect(
    &thread,
    &cmn::RemoteProjectBuilderThread::finished,
    [&success, &message, &loop](bool _success, const QString& _message)
    {
      success = _success;
      message = _message;
      loop.quit();
    });

  // イベントループを回しながらスレッドが終了するまで待機
  thread.start();
  loop.exec();
  thread.wait();

  // 別スレッドの結果を返す
  if (success) {
    return {};
  }
  else {
    return std::unexpected(message);
  }
}
}  // namespace cmn
}  // namespace gui
