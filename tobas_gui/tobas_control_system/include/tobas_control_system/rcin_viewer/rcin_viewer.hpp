// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./throttles_viewer.hpp"
#include "./toggles_viewer.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace rcin
{
class RCInputViewerWidget : public QWidget
{
  Q_OBJECT

public:
  explicit RCInputViewerWidget(const RosQtBridge& bridge);

  void reset();

private:
  ThrottlesViewer* throttles_viewer_;
  TogglesViewer* toggles_viewer_;
};
}  // namespace rcin
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
