// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

#include <tobas_rviz_wrapper/rviz.hpp>

namespace tobas
{
namespace gui
{
namespace ub
{
class URDFBuilder : public QWidget
{
  Q_OBJECT

public:
  explicit URDFBuilder();

private:
  rviz::RvizFrameManager rviz_manager_;
};
}  // namespace ub
}  // namespace gui
}  // namespace tobas
