#pragma once

#include <optional>
#include <QWidget>
#include <QPainter>
#include <QPaintEvent>

namespace qt
{
class PositionBarWidget : public QWidget
{
  Q_OBJECT

  using super = QWidget;

  static constexpr int kDefaultLineWidth = 3;
  static constexpr int kDefaultTextPSize = 10;

public:
  explicit PositionBarWidget(
    bool fill_range = true,
    double minimum = 0.,
    double maximum = 0.,
    int line_width = kDefaultLineWidth,
    int text_psize = kDefaultTextPSize,
    QWidget* parent = nullptr);

  virtual void drawRange(QPainter& painter, double lower, double upper) = 0;
  virtual void drawValue(QPainter& painter, double value) = 0;
  virtual void drawText(QPainter& painter, const QString& text) = 0;

  void paintEvent(QPaintEvent* event) override;

  double getMinimum() const;
  double getMaximum() const;
  int getLineWidth() const;
  int getTextPSize() const;
  const QString& getText() const;
  double getValue() const;
  double getLower() const;
  double getUpper() const;
  double getMiddle() const;

  void setMinimum(double minimum);
  void setMaximum(double maximum);
  void setLineWidth(int line_width);
  void setTextPSize(int text_psize);
  void setText(const QString& text);
  void setValue(double value);
  void setLower(double lower);
  void setUpper(double upper);

  void clear();

private:
  bool fill_range_;
  double minimum_;
  double maximum_;
  int line_width_;
  int text_psize_;

  std::optional<QString> text_;
  std::optional<double> value_;
  std::optional<double> lower_;
  std::optional<double> upper_;
};

class HPositionBarWidget : public PositionBarWidget
{
  Q_OBJECT

  void drawRange(QPainter& painter, double lower, double upper) override;
  void drawValue(QPainter& painter, double value) override;
  void drawText(QPainter& painter, const QString& text) override;
};

class VPositionBarWidget : public PositionBarWidget
{
  Q_OBJECT

  void drawRange(QPainter& painter, double lower, double upper) override;
  void drawValue(QPainter& painter, double value) override;
  void drawText(QPainter& painter, const QString& text) override;
};
}  // namespace qt
