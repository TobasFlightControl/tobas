#pragma once

#include <rclcpp/node.hpp>
#include <QThread>

namespace gui
{
namespace log
{
class RecordStartThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit RecordStartThread(rclcpp::Node::SharedPtr node);

  void run() override;

  void setNamespace(const std::string& ns);
  void setLogName(const std::string& log_name);

private:
  const rclcpp::Node::SharedPtr node_;

  std::string ns_;
  std::string log_name_;
};
}  // namespace log
}  // namespace gui
