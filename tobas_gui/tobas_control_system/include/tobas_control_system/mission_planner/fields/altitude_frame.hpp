#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "../altitude_frame.hpp"
#include "./base.hpp"

namespace gui
{
namespace gcs
{
namespace field
{
class AltitudeFrameWidget : public BaseField
{
public:
  explicit AltitudeFrameWidget();

  const char* label() const override;

  AltitudeFrame value() const;
  void setValue(AltitudeFrame value);

private:
  qt::ComboBox* combobox_;
};
}  // namespace field
}  // namespace gcs
}  // namespace gui
