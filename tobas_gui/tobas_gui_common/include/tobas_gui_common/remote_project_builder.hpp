#pragma once

#include <QThread>
#include <expected>
#include <filesystem>

#include <tobas_ssh_client/ssh_client.hpp>

namespace gui
{
namespace cmn
{
class RemoteProjectBuilder
{
public:
  explicit RemoteProjectBuilder(rclcpp::Node::SharedPtr node);

  bool build(const std::filesystem::path& remote_proj_path);

  const std::string& getOutput() const;
  const char* getErrorMessage() const;

private:
  const rclcpp::Node::SharedPtr node_;
  ssh::SSHClient ssh_client_;

  std::string output_;
};

class RemoteProjectBuilderThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit RemoteProjectBuilderThread(rclcpp::Node::SharedPtr node, const std::filesystem::path& proj_path);

  void run() override;

private:
  const std::filesystem::path proj_path_;

  RemoteProjectBuilder builder_;
};

/* 別スレッドでリモートプロジェクトをビルドする． */
std::expected<void, QString>
buildRemoteProjectBackground(rclcpp::Node::SharedPtr node, const std::filesystem::path& proj_path);
}  // namespace cmn
}  // namespace gui
