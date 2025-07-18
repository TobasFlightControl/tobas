#pragma once

#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_qt_tools/widgets/framed_label.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>

#include "./robot_info.hpp"

namespace gui
{
namespace sa
{
class RobotPropertiesWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = RobotPropertiesWidget;
  using super = qt::ScrollArea;

public:
  explicit RobotPropertiesWidget(const RobotInfo& robot);

  void updateInternalDataStructures();

private:
  const RobotInfo& robot_;

  kdl::TreeMassHolder mass_holder_;

  qt::FramedLabel* mass_;
  qt::FramedLabel* frame_type_;
};
}  // namespace sa
}  // namespace gui
