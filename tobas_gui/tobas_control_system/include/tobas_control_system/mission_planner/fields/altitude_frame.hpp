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
class AltitudeFrameWidget : public BaseField
{
public:
  explicit AltitudeFrameWidget();

  const char* label() const override;

  tobas::mission::AltitudeFrame value() const;
  void setValue(tobas::mission::AltitudeFrame value);

private:
  qt::ComboBox* combobox_;
};
}  // namespace field
}  // namespace ctrl
}  // namespace gui
