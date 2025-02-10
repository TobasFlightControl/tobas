#pragma once

#include <rclcpp/node.hpp>
#include <QThread>

namespace gui
{
namespace log
{
class RecordStopThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit RecordStopThread(rclcpp::Node::SharedPtr node);

  void run() override;

  void setNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  std::string ns_;
};
}  // namespace log
}  // namespace gui
