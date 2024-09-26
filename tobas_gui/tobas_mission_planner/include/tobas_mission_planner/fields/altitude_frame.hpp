#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "./base.hpp"
#include "../altitude_frame.hpp"

namespace gui
{
namespace mission_planner
{
namespace field
{
class AltitudeFrameWidget : public BaseField
{
public:
  explicit AltitudeFrameWidget();

  const char* label() const override;

  altitude_frame_t value() const;
  void setValue(altitude_frame_t value);

private:
  qt::ComboBox* combobox_;
};
}  // namespace field
}  // namespace mission_planner
}  // namespace gui
