// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_bootmedia_config/base.hpp"

namespace tobas::gui::bm
{
BaseConfigWidget::BaseConfigWidget()
{
  // Fix the `QTabWidget` background to the base color because the default background is too white.
  setAutoFillBackground(true);
  auto pal = palette();
  pal.setColor(QPalette::Window, pal.color(QPalette::Base));
}
}  // namespace tobas::gui::bm
