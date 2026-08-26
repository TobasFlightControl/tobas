// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QEventLoop>
#include <QObject>
#include <QTimer>

#include "./bridge.hpp"

namespace tobas
{
namespace gui
{
template <typename MsgType, auto SignalType>
typename MsgType::ConstSharedPtr waitForMessage(RosQtBridge& bridge, int timeout_ms)
{
  typename MsgType::ConstSharedPtr res;

  QEventLoop event_loop;
  QTimer timeout_timer;
  timeout_timer.setSingleShot(true);

  const auto connection = QObject::connect(
    &bridge,
    SignalType,
    &event_loop,
    [&res, &event_loop](const typename MsgType::ConstSharedPtr& msg)
    {
      res = msg;
      event_loop.quit();
    },
    Qt::QueuedConnection);
  QObject::connect(&timeout_timer, &QTimer::timeout, &event_loop, &QEventLoop::quit);

  timeout_timer.start(timeout_ms);
  event_loop.exec(QEventLoop::AllEvents | QEventLoop::ExcludeUserInputEvents);

  QObject::disconnect(connection);
  return res;
}
}  // namespace gui
}  // namespace tobas
