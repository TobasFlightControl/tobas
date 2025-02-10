#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "./general/general.hpp"
#include "./esc.hpp"
#include "./motor.hpp"
#include "./propeller.hpp"
#include "./aerodynamics/aerodynamics.hpp"
#include "./speed_limit/speed_limit.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
class SelectedLinkWidget : public QWidget
{
  Q_OBJECT

  static constexpr int kButtonWidth = 125;
  static constexpr int kButtonHeight = 50;
  static constexpr int kTabWidth = 135;
  static constexpr int kTabHeight = 45;

Q_SIGNALS:
  void copyFromLeftButtonClicked();
  void copyToAllButtonClicked();
  void channelChanged(int channel);
  void isTiltStateChanged(bool is_tilt);
  void tiltJointNameChanged(const QString& joint_name);

public:
  explicit SelectedLinkWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot, const QString& link_name);

  bool isValid();
  void copyFrom(const SelectedLinkWidget* src);

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  const GeneralWidget* general() const;
  const ESCWidget* esc() const;
  const MotorWidget* motor() const;
  const PropellerWidget* propeller() const;
  const AerodynamicsWidget* aerodynamics() const;
  const SpeedLimitWidget* speedLimit() const;

private:
  qt::TabWidget* tabs_;

  QPushButton* copy_from_left_button_;
  QPushButton* copy_to_all_button_;

  GeneralWidget* general_;
  ESCWidget* esc_;
  MotorWidget* motor_;
  PropellerWidget* propeller_;
  AerodynamicsWidget* aerodynamics_;
  SpeedLimitWidget* speed_limit_;
};
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
