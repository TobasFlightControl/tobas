#pragma once

#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_qt_tools/widgets/framed_label.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>

#include "./frame_type.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class RobotPropertiesWidget : public tobas::qt::ScrollArea
{
  Q_OBJECT

  using self = RobotPropertiesWidget;
  using super = tobas::qt::ScrollArea;

public:
  explicit RobotPropertiesWidget(const kdl::Tree& tree);

  void updateInternalDataStructures();

  void setFrameType(const FrameType& type);

private:
  kdl::TreeMassHolder mass_holder_;

  tobas::qt::FramedLabel* frame_type_;
  tobas::qt::FramedLabel* mass_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
