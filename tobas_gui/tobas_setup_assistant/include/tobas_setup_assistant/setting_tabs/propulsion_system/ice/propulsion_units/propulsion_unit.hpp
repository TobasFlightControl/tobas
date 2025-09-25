#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "./aerodynamics.hpp"
#include "./propeller.hpp"
#include "./transmission.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class PropulsionUnitWidget : public QWidget
{
  Q_OBJECT

  static constexpr int kButtonWidth = 125;
  static constexpr int kButtonHeight = 50;
  static constexpr int kTabWidth = 120;
  static constexpr int kTabHeight = 40;

Q_SIGNALS:
  void copyToAllButtonClicked();

public:
  explicit PropulsionUnitWidget(rclcpp::Node::SharedPtr node);

  bool isValid();
  void copyFrom(const PropulsionUnitWidget* src);

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  const TransmissionWidget* transmission() const;
  const PropellerWidget* propeller() const;
  const AerodynamicsWidget* aerodynamics() const;

private:
  qt::TabWidget* tabs_;

  QPushButton* copy_to_all_btn_;

  TransmissionWidget* transmission_;
  PropellerWidget* propeller_;
  AerodynamicsWidget* aerodynamics_;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
