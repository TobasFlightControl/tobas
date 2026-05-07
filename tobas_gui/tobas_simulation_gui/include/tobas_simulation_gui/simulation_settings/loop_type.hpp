// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QButtonGroup>
#include <QRadioButton>

namespace tobas
{
namespace gui
{
namespace sim
{
enum LoopType
{
  SITL,
  HITL,
};

class LoopTypeWidget : public QWidget
{
  Q_OBJECT

  using self = LoopTypeWidget;
  using super = QWidget;

public:
  explicit LoopTypeWidget();

  LoopType loopType() const;

private:
  QButtonGroup* btn_group_;

  QRadioButton* sitl_btn_;
  QRadioButton* hitl_btn_;
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
