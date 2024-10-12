#pragma once

#include <cmath>
#include <QWidget>
#include <QTimer>
#include <QColor>

namespace qt
{
class WaitSpinnerWidget : public QWidget
{
  Q_OBJECT

  using self = WaitSpinnerWidget;
  using super = QWidget;

public:
  /**
   * @brief Constructor for "standard" widget behaviour.
   * Use this constructor if you wish to, e.g. embed your widget in another.
   */
  WaitSpinnerWidget(QWidget* parent = nullptr, bool center_on_parent = true, bool disable_parent_when_spinning = true);

  /**
   * @brief Constructor.
   * Use this constructor to automatically create a modal ("blocking") spinner on top of the calling widget/window.
   * If a valid parent widget is provided,
   * "center_on_parent" will ensure that the spinner automatically centers itself on it,
   * if not, "center_on_parent" is ignored.
   */
  WaitSpinnerWidget(
    Qt::WindowModality modality,
    QWidget* parent = nullptr,
    bool center_on_parent = true,
    bool disable_parent_when_spinning = true);

  void setColor(QColor color);
  void setRoundness(double roundness);
  void setMinimumTrailOpacity(double min_trail_opacity);
  void setTrailFadePercentage(double trail);
  void setRevolutionsPerSecond(double revolution_per_sec);
  void setNumberOfLines(int lines);
  void setLineLength(int length);
  void setLineWidth(int width);
  void setInnerRadius(int radius);

  QColor color() const;
  double roundness() const;
  double minimumTrailOpacity() const;
  double trailFadePercentage() const;
  double revolutionsPersSecond() const;
  int numberOfLines() const;
  int lineLength() const;
  int lineWidth() const;
  int innerRadius() const;

  bool isSpinning() const;

public Q_SLOTS:
  void start();
  void stop();

protected:
  void paintEvent(QPaintEvent* event);

private Q_SLOTS:
  void rotate();

private:
  const bool center_on_parent_;
  const bool disable_parent_when_spinning_;

  QColor color_ = Qt::black;
  double roundness_ = 100.;  // 0..100
  double min_trail_opacity_ = M_PI;
  double trail_fade_perc_ = 80.;
  double revolutions_per_sec_ = M_PI_2;
  int num_lines_ = 20;
  int line_length_ = 10;
  int line_width_ = 2;
  int inner_radius_ = 10;
  int cur_counter_ = 0;
  bool is_spinning_ = false;

  QTimer* timer_;

  static int lineCountDistanceFromPrimary(int current, int primary, int tobas_num_lines);
  static QColor
  currentLineColor(int distance, int tobas_num_lines, double trail_fade_perc, double min_opacity, QColor color);

  void initialize();
  void updateSize();
  void updateTimer();
  void updatePosition();
};
}  // namespace qt
