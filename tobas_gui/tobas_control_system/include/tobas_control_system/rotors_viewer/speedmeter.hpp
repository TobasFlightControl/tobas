// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QtQuickWidgets/QQuickWidget>

namespace tobas::gui::ctrl
{
class SpeedmeterWidget : public QQuickWidget
{
  Q_OBJECT

public:
  explicit SpeedmeterWidget();

  QString getBackgroundColor() const;
  double getMaximumValue() const;
  double getMinimumValue() const;
  double getStepSize() const;
  double getValue() const;
  QString getTopText() const;
  QString getBottomText() const;

  void setBackgroundColor(const QString& _text);
  void setMaximumValue(double _value);
  void setMinimumValue(double _value);
  void setStepSize(double _value);
  void setValue(double _value);
  void setTopText(const QString& _text);
  void setBottomText(const QString& _text);

private:
  QObject* getRectangleObject() const;
  QObject* getGaugeObject() const;
  QObject* getTopTextObject() const;
  QObject* getBottomTextObject() const;
};
}  // namespace tobas::gui::ctrl
