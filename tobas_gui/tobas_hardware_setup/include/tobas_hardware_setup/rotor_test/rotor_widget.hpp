#pragma once

#include <QLabel>
#include <QLineEdit>

#include <tobas_qt_tools/widgets/slider.hpp>

#include "./speedmeter.hpp"

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
  SpeedmeterWidget* cur_rpm_bar_;
  qt::Slider* tar_rpm_slider_;
  qt::Slider* gain_slider_;
  QLineEdit* cur_rpm_box_;
  QLineEdit* tar_rpm_box_;
  QLineEdit* gain_box_;

private Q_SLOTS:
  void onTargetRPMChanged(int rpm);
  void onGainChanged(int gain);
};
}  // namespace hardware_setup
}  // namespace gui
