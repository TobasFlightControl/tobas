// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <qwt/qwt_thermo.h>
#include <QHBoxLayout>
#include <QLabel>

#include <tobas_qt_tools/widgets/framed_label.hpp>

namespace tobas::gui::ctrl
{
class SpeedmeterWidget : public QWidget
{
  Q_OBJECT

public:
  explicit SpeedmeterWidget();

  QSize minimumSizeHint() const override;

  QString getBackgroundColor() const;
  double getMaximumValue() const;
  double getMinimumValue() const;
  double getValue() const;
  QString getTopText() const;
  QString getBottomText() const;

  void setBackgroundColor(const QString& _text);
  void setMaximumValue(double _value);
  void setMinimumValue(double _value);
  void setValue(double _value);
  void setTopText(const QString& _text);
  void setBottomText(const QString& _text);

protected:
  bool eventFilter(QObject* _watched, QEvent* _event) override;
  void resizeEvent(QResizeEvent* _event) override;

private:
  QLabel* top_text_;
  qt::FramedLabel* bottom_text_;
  QwtThermo* gauge_;
  QHBoxLayout* gauge_row_;
  QString top_text_value_;
  QString bottom_text_value_;

  double step_size_ = 1.0;
  double value_ = 0.0;

  void updateGaugeValue();
  void updateGaugeAlignment();
  void updateLabels();
};
}  // namespace tobas::gui::ctrl
