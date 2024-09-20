#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/wait_spinner.hpp>

#include "../base.hpp"
#include "./thread.hpp"

namespace gui
{
namespace hardware_setup
{
class AccelCalibrationWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = AccelCalibrationWidget;
  using super = BaseHardwareSetupWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit AccelCalibrationWidget(rclcpp::Node::SharedPtr node);

  const char* name() const override;
  const char* title() const override;

  void onInit() override;

  void setNamespace(const std::string& ns);

private:
  QPushButton* start_button_;

  qt::WaitSpinnerWidget spinner_;
  AccelCalibrationThread thread_;

private Q_SLOTS:
  void onStartButtonClicked();
  void onCalibrationFinished(bool success, const QString& output);
};
}  // namespace hardware_setup
}  // namespace gui
