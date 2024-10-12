#pragma once

#include <filesystem>

#include <tobas_ssh_client/ssh_client.hpp>

namespace gui
{
namespace common
{
class RemotePackageBuilder
{
public:
  explicit RemotePackageBuilder(rclcpp::Node::SharedPtr node);

  bool build(const std::filesystem::path& remote_tbs_path);

  const std::string& getOutput() const;
  const char* getErrorMessage() const;

private:
  const rclcpp::Node::SharedPtr node_;
  ssh::SSHClient ssh_client_;

  std::string output_;
};
}  // namespace common
}  // namespace gui
