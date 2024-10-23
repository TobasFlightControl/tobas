#pragma once

#include <QWidget>

namespace qt
{
class PositionBarWidget : public QWidget
{
  Q_OBJECT

  using super = QWidget;

public:
  explicit PositionBarWidget(QWidget* parent = nullptr);
  explicit PositionBarWidget(double minimum, double maximum, QWidget* parent = nullptr);

  bool hasText() const;
  bool hasValue() const;
  bool hasLower() const;
  bool hasUpper() const;

  double getMinimum() const;
  double getMaximum() const;
  int getLineWidth() const;
  int getTextPSize() const;
  const QString& getText() const;
  double getValue() const;
  double getLower() const;
  double getUpper() const;
  double getMiddle() const;
  double getRange() const;

  void setMinimum(double minimum);
  void setMaximum(double maximum);
  void setFillRange(bool fill_range);
  void setLineWidth(int line_width);
  void setTextPSize(int text_psize);
  void setFillColor(Qt::GlobalColor color);
  void setLimitLineColor(Qt::GlobalColor color);
  void setValueLineColor(Qt::GlobalColor color);
  void setText(const QString& text);
  void setValue(double value);
  void setLower(double lower);
  void setUpper(double upper);

  void clear();

protected:
  void paintEvent(QPaintEvent* event) override;

  double minimum_ = 0.;
  double maximum_ = 0.;
  bool fill_range_ = true;
  int line_width_ = 3;
  int text_psize_ = 10;
  Qt::GlobalColor fill_color_ = Qt::green;
  Qt::GlobalColor limit_line_color_ = Qt::black;
  Qt::GlobalColor value_line_color_ = Qt::red;

  std::optional<QString> text_;
  std::optional<double> value_;
  std::optional<double> lower_;
  std::optional<double> upper_;

  virtual void drawRange(QPainter& painter, double lower, double upper) = 0;
  virtual void drawValue(QPainter& painter, double value) = 0;
  virtual void drawText(QPainter& painter, const QString& text) = 0;
};

class HPositionBarWidget : public PositionBarWidget
{
  Q_OBJECT

public:
  using PositionBarWidget::PositionBarWidget;

private:
  void drawRange(QPainter& painter, double lower, double upper) override;
  void drawValue(QPainter& painter, double value) override;
  void drawText(QPainter& painter, const QString& text) override;
};

class VPositionBarWidget : public PositionBarWidget
{
  Q_OBJECT

public:
  using PositionBarWidget::PositionBarWidget;

private:
  void drawRange(QPainter& painter, double lower, double upper) override;
  void drawValue(QPainter& painter, double value) override;
  void drawText(QPainter& painter, const QString& text) override;
};
}  // namespace qt
