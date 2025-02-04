#pragma once

#include <QLabel>
#include <qwt/qwt_thermo.h>
#include <qwt/qwt_slider.h>

#include <tobas_qt_tools/widgets/framed_label.hpp>

namespace gui
{
namespace hardware_setup
{
class RotorWidget : public QWidget
{
  Q_OBJECT

  using self = RotorWidget;
  using super = QWidget;

Q_SIGNALS:
  void targetRPMChanged(int rpm);
  void gainChanged(int gain);

public:
  explicit RotorWidget();

  void reset();

  QString getText() const;
  int getCurrentRPM() const;
  int getTargetRPM() const;
  int getGain() const;

  void setText(const QString& text);
  void setMaximumRPM(int rpm);
  void setCurrentRPM(int rpm);
  void setTargetRPM(int rpm);
  void setGain(int gain);

private:
  QLabel* text_;
  QwtThermo* cur_rpm_meter_;
  QwtSlider* tar_rpm_slider_;
  QwtSlider* gain_slider_;
  qt::FramedLabel* cur_rpm_box_;
  qt::FramedLabel* tar_rpm_box_;
  qt::FramedLabel* gain_box_;

  void setCurrentRPMBox(int rpm);
  void setTargetRPMBox(int rpm);
  void setGainBox(int gain);

  static QString rpmToText(int rpm);

private Q_SLOTS:
  void onTargetRPMChanged(int rpm);
  void onGainChanged(int gain);
};
}  // namespace hardware_setup
}  // namespace gui
