#pragma once

#include <tobas_mission_items/mission_items.hpp>
#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace ctrl
{
namespace field
{
class AltitudeFrameWidget : public FieldWidget<tobas::mission::AltitudeFrame>
{
public:
  explicit AltitudeFrameWidget();

  const char* label() const override;

  tobas::mission::AltitudeFrame getValue() const override;
  void setValue(tobas::mission::AltitudeFrame value) override;

private:
  qt::ComboBox* combobox_;
};
}  // namespace field
}  // namespace ctrl
}  // namespace gui
