#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "./aerodynamics/aerodynamics.hpp"
#include "./esc.hpp"
#include "./motor.hpp"
#include "./propeller.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
class PropulsionUnitWidget : public QWidget
{
  Q_OBJECT

  static constexpr int kButtonWidth = 125;
  static constexpr int kButtonHeight = 50;
  static constexpr int kTabWidth = 135;
  static constexpr int kTabHeight = 45;

Q_SIGNALS:
  void copyFromLeftButtonClicked();
  void copyToAllButtonClicked();

public:
  explicit PropulsionUnitWidget(rclcpp::Node::SharedPtr node, const QString& link_name);

  bool isValid();
  void copyFrom(const PropulsionUnitWidget* src);

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  const ESCWidget* esc() const;
  const MotorWidget* motor() const;
  const PropellerWidget* propeller() const;
  const AerodynamicsWidget* aerodynamics() const;

private:
  qt::TabWidget* tabs_;

  QPushButton* copy_from_left_button_;
  QPushButton* copy_to_all_button_;

  ESCWidget* esc_;
  MotorWidget* motor_;
  PropellerWidget* propeller_;
  AerodynamicsWidget* aerodynamics_;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
