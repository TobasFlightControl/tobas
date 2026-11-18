// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <utility>

#include <QLabel>
#include <QWidget>

#include <tobas_qt_tools/widgets/spin_box.hpp>

namespace tobas::gui::sa
{
class RatioWidget : public QWidget
{
  Q_OBJECT

  using self = RatioWidget;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(const std::pair<int, int>& _value);

public:
  explicit RatioWidget(QWidget* _parent = nullptr);

  std::pair<int, int> value() const;
  void setValue(const std::pair<int, int>& _value);

  void setMinimum(int _minimum);
  void setMaximum(int _maximum);
  void setLeftText(const QString& _text);
  void setRightText(const QString& _text);

private:
  qt::SpinBox* left_value_;
  qt::SpinBox* right_value_;
  QLabel* left_text_;
  QLabel* right_text_;

private Q_SLOTS:
  void onValueChanged();
};
}  // namespace tobas::gui::sa
