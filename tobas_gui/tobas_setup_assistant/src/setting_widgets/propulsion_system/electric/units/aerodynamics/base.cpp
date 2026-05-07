// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/base.hpp"

#include <QTimer>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
AerodynamicsWidget_Base::AerodynamicsWidget_Base()
{
  rows_ = new QVBoxLayout();
  setLayout(rows_);

  description_ = new qt::DescriptionWidget("", cmn::kBodyPSize);
  rows_->addWidget(description_);

  QTimer::singleShot(0, this, &AerodynamicsWidget_Base::initialize);
}

void AerodynamicsWidget_Base::initialize()
{
  description_->setText(description());
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
