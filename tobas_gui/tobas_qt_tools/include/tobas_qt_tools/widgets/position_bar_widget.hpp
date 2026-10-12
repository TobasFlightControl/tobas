// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QColor>
#include <QWidget>

namespace tobas
{
namespace qt
{
class PositionBarWidget : public QWidget
{
  Q_OBJECT

  using super = QWidget;

public:
  explicit PositionBarWidget(QWidget* _parent = nullptr);
  explicit PositionBarWidget(double _minimum, double _maximum, QWidget* _parent = nullptr);

  double getMinimum() const;
  double getMaximum() const;
  int getLineWidth() const;
  int getTextPSize() const;
  std::optional<QString> getCenterText() const;
  std::optional<QString> getLowerText() const;
  std::optional<QString> getUpperText() const;
  std::optional<double> getValue() const;
  std::optional<double> getLower() const;
  std::optional<double> getUpper() const;
  std::optional<double> getMiddle() const;
  std::optional<double> getRange() const;

  void setMinimum(double _minimum);
  void setMaximum(double _maximum);
  void setLineWidth(int _line_width);
  void setTextPSize(int _text_psize);
  void setFillColor(const QColor& _color);
  void setLimitLineColor(const QColor& _color);
  void setValueLineColor(const QColor& _color);
  void setCenterText(const QString& _text);
  void setLowerText(const QString& _text);
  void setUpperText(const QString& _text);
  void setValue(double _value);
  void setLower(double _lower);
  void setUpper(double _upper);

  void clear();
  void updateRangeFromValue();

protected:
  void paintEvent(QPaintEvent* _event) override;

  double minimum_ = 0.0;
  double maximum_ = 1.0;
  int line_width_ = 3;
  int text_psize_ = 10;
  QColor fill_color_ = Qt::green;
  QColor limit_line_color_ = Qt::black;
  QColor value_line_color_ = Qt::red;

  std::optional<QString> center_text_;
  std::optional<QString> lower_text_;
  std::optional<QString> upper_text_;
  std::optional<double> value_;
  std::optional<double> lower_;
  std::optional<double> upper_;

  virtual void drawRange(QPainter& _painter, double _lower, double _upper) = 0;
  virtual void drawValue(QPainter& _painter, double _value) = 0;
  virtual void drawCenterText(QPainter& _painter, const QString& _text) = 0;
  virtual void drawLowerText(QPainter& _painter, const QString& _text) = 0;
  virtual void drawUpperText(QPainter& _painter, const QString& _text) = 0;
};

class HPositionBarWidget : public PositionBarWidget
{
  Q_OBJECT

public:
  using PositionBarWidget::PositionBarWidget;

protected:
  void drawRange(QPainter& _painter, double _lower, double _upper) override;
  void drawValue(QPainter& _painter, double _value) override;
  void drawCenterText(QPainter& _painter, const QString& _text) override;
  void drawLowerText(QPainter& _painter, const QString& _text) override;
  void drawUpperText(QPainter& _painter, const QString& _text) override;

private:
  void drawTextCommon(QPainter& _painter);
};

class VPositionBarWidget : public PositionBarWidget
{
  Q_OBJECT

public:
  using PositionBarWidget::PositionBarWidget;

protected:
  void drawRange(QPainter& _painter, double _lower, double _upper) override;
  void drawValue(QPainter& _painter, double _value) override;
  void drawCenterText(QPainter& _painter, const QString& _text) override;
  void drawLowerText(QPainter& _painter, const QString& _text) override;
  void drawUpperText(QPainter& _painter, const QString& _text) override;

private:
  void drawTextCommon(QPainter& _painter);
};
}  // namespace qt
}  // namespace tobas
