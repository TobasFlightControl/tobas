// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QToolButton>

namespace tobas::gui::gcs
{
class AppButton : public QToolButton
{
  Q_OBJECT

public:
  explicit AppButton(const QString& _text, const QString& _icon_path);
};
}  // namespace tobas::gui::gcs
