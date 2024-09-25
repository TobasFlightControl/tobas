#include <magic_enum.hpp>
#include <QHBoxLayout>

#include "tobas_mission_planner/fields/altitude_frame.hpp"

namespace gui
{
namespace mission_planner
{
namespace field
{
AltitudeFrameWidget::AltitudeFrameWidget()
{
  combobox_ = new qt::ComboBox();
  for (const auto& alt_frame : magic_enum::enum_values<altitude_frame_t>())
    combobox_->addItem(altFrameToText(alt_frame));

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(combobox_);

  connect(combobox_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &BaseField::updated);
}

const char* AltitudeFrameWidget::label() const
{
  return "Altitude Frame";
}

altitude_frame_t AltitudeFrameWidget::value() const
{
  return textToAltFrame(combobox_->currentText().toUtf8());
}
}  // namespace field
}  // namespace mission_planner
}  // namespace gui
