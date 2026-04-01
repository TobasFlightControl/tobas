// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QThread>
#include <rclcpp/node.hpp>

namespace tobas
{
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
}  // namespace tobas
