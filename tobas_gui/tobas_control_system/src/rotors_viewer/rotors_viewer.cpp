#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_control_system/rotors_viewer/rotors_viewer.hpp"

namespace gui
{
namespace control_system
{
RotorsViewerWiddget::RotorsViewerWiddget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone)
  : node_(node), drone_(drone)
{
  cols_ = new QHBoxLayout();
  setLayout(cols_);
}

void RotorsViewerWiddget::updateInternalDataStructures()
{
  meters_.clear();
  qt::clearLayout(cols_);

  for (const auto& rotor : drone_.rotors)
  {
    const auto meter = new SpeedmeterWidget();
    meter->setMaximumValue(tobas_std::rps2rpm(rotor.max_rot_speed));
    meter->setTopText(rotor.link_name.c_str());

    meters_.push_back(meter);
    cols_->addWidget(meter);
  }
}
}  // namespace control_system
}  // namespace gui
