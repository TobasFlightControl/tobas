#include "tobas_setup_assistant/robot_properties.hpp"

#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_std_tools/check.hpp>

namespace gui
{
namespace sa
{
RobotPropertiesWidget::RobotPropertiesWidget(const kdl::Tree& tree) : mass_holder_(tree)
{
  frame_type_ = new qt::FramedLabel();
  mass_ = new qt::FramedLabel();

  const auto form = new qt::FormLayout();
  form->setHorizontalSpacing(30);
  setLayout(form);
  form->addRow("Frame Type", frame_type_);
  form->addRow("Total Mass", mass_);
}

void RobotPropertiesWidget::updateInternalDataStructures()
{
  TOBAS_CHECK(mass_holder_.updateInternalDataStructures());
  mass_->setText(QString::number(mass_holder_.getMass()) + " kg");
}

void RobotPropertiesWidget::setFrameType(const FrameType& type)
{
  const auto text = QString::fromStdString(textFromEnum(type));
  frame_type_->setText(text);
}
}  // namespace sa
}  // namespace gui
