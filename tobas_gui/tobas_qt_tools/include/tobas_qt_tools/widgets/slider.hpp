#pragma once

#include <QSlider>

namespace qt
{
/**
 * ===== QSliderとの違い =====
 * - マウスホイールイベントを無効化
 */
class Slider : public QSlider
{
  Q_OBJECT

  using self = Slider;
  using super = QSlider;

public:
  using QSlider::QSlider;

protected:
  void wheelEvent(QWheelEvent* event) override;
};
}  // namespace qt
