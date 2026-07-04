// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QColor>
#include <QLabel>

#include <tobas_qt_tools/widgets/framed_label.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
class StatusWidget : public QWidget
{
  Q_OBJECT

  static constexpr int kStatusWidth = 40;
  static constexpr int kTextPSize = 12;

public:
  enum Status
  {
    PASSED = 0,
    FAILED = 1,
    IGNORED = 2,
    UNKNOWN = 3,
  };

  explicit StatusWidget(const QString& text);

  void reset();

  void setStatus(Status status);
  void setStatus(uint8_t status);
  void setStatus(bool ok);

private:
  qt::FramedLabel* status_;
  QLabel* label_;

  void setStatusText(const QString& text, const QColor& color);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
