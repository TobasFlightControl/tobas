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
namespace setup_assistant
{
namespace propulsion_system
{
class SelectedLinkWidget : public QWidget
{
  Q_OBJECT

  static constexpr int kButtonWidth = 120;
  static constexpr int kButtonHeight = 50;
  static constexpr int kTabWidth = 135;
  static constexpr int kTabHeight = 45;

Q_SIGNALS:
  void copyFromLeftButtonClicked();
  void copyToAllButtonClicked();

public:
  explicit SelectedLinkWidget(rclcpp::Node::SharedPtr node);

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
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
