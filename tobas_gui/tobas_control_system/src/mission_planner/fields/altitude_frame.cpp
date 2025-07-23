#include "tobas_control_system/mission_planner/fields/altitude_frame.hpp"

#include <QHBoxLayout>
#include <magic_enum/magic_enum.hpp>

namespace gui
{
namespace gcs
{
namespace field
{
AltitudeFrameWidget::AltitudeFrameWidget()
{
  combobox_ = new qt::ComboBox();
  combobox_->addItem(altFrameToText(AltitudeFrame::kRelativeToHome));  // TODO: 他の選択肢も選べるようにする
  // for (const auto& alt_frame : magic_enum::enum_values<AltitudeFrame>())
  //   combobox_->addItem(altFrameToText(alt_frame));

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(combobox_);

  connect(combobox_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &BaseField::updated);
}

const char* AltitudeFrameWidget::label() const
{
  return "Altitude Frame";
}

AltitudeFrame AltitudeFrameWidget::value() const
{
  return textToAltFrame(combobox_->currentText().toUtf8());
}

void AltitudeFrameWidget::setValue(AltitudeFrame value)
{
  combobox_->setCurrentText(altFrameToText(value));
}
}  // namespace field
}  // namespace gcs
}  // namespace gui
