#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "./general.hpp"
#include "./transmission.hpp"
#include "./propeller.hpp"
#include "./hardware_interface.hpp"
#include "./aerodynamics/aerodynamics.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
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

public:
  explicit SelectedLinkWidget();

  bool isValid();
  void copyFrom(const SelectedLinkWidget* src);

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  const GeneralWidget* general() const;
  const TransmissionWidget* transmission() const;
  const PropellerWidget* propeller() const;
  const VPitchHardwareIfaceWidget* hardwareIface() const;
  const AerodynamicsWidget* aerodynamics() const;

private:
  qt::TabWidget* tabs_;

  QPushButton* copy_from_left_button_;
  QPushButton* copy_to_all_button_;

  GeneralWidget* general_;
  TransmissionWidget* transmission_;
  PropellerWidget* propeller_;
  VPitchHardwareIfaceWidget* hw_iface_;
  AerodynamicsWidget* aerodynamics_;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
