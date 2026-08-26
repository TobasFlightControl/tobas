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
namespace detail
{
/**
 * Extract the argument type from a single-argument Qt signal.
 * For example, `void received(const Msg::ConstSharedPtr&)` yields `Msg::ConstSharedPtr`.
 */
template <typename>
struct SignalTraits;

template <typename Class, typename Arg>
struct SignalTraits<void (Class::*)(const Arg&)>
{
  using Argument = Arg;
};
}  // namespace detail

template <auto SignalType>
auto waitForMessage(RosQtBridge& bridge, int timeout_ms)
{
  // Derive the message pointer type from the signal so callers only need to specify the signal itself.
  using MsgPtr = typename detail::SignalTraits<decltype(SignalType)>::Argument;

  MsgPtr res;

  QEventLoop event_loop;
  QTimer timeout_timer;
  timeout_timer.setSingleShot(true);

  const auto connection = QObject::connect(
    &bridge,
    SignalType,
    &event_loop,
    [&res, &event_loop](const MsgPtr& msg)
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
