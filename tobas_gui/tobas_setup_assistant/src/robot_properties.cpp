#include "tobas_setup_assistant/robot_properties.hpp"

#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_std_tools/check.hpp>

namespace gui
{
namespace sa
{
RobotPropertiesWidget::RobotPropertiesWidget(const RobotInfo& robot) : robot_(robot), mass_holder_(robot.tree())
{
  mass_ = new qt::FramedLabel();
  frame_type_ = new qt::FramedLabel();

  const auto form = new qt::FormLayout();
  form->setHorizontalSpacing(30);
  setLayout(form);
  form->addRow("Total Mass", mass_);
  form->addRow("Frame Type", frame_type_);
}

void RobotPropertiesWidget::updateInternalDataStructures()
{
  TOBAS_CHECK(mass_holder_.updateInternalDataStructures());
  mass_->setText(QString::number(mass_holder_.getMass()) + " kg");

  // TODO: Set frame type
}
}  // namespace sa
}  // namespace gui
