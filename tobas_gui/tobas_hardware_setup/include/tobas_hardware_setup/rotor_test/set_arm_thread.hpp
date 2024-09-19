#pragma once

#include <rclcpp/node.hpp>
#include <QThread>

namespace gui
{
namespace hardware_setup
{
class SetArmThread : public QThread
{
  Q_OBJECT

  using self = SetArmThread;
  using super = QThread;

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit SetArmThread(rclcpp::Node::SharedPtr node, bool arming = false);

  void run() override;

  void setNamespace(const std::string& ns);
  bool setArming(bool arming);

private:
  const rclcpp::Node::SharedPtr node_;

  std::string ns_;
  bool arming_;
};
}  // namespace hardware_setup
}  // namespace gui
