// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/base.hpp"

#include <QFrame>

namespace tobas::gui::sa::propulsion::ice
{
BaseSelectedLinkSettingWidget::BaseSelectedLinkSettingWidget()
{
  setFrameShape(QFrame::NoFrame);
}
}  // namespace tobas::gui::sa::propulsion::ice
