#pragma once

#include "./base_setting.hpp"
#include "../param_getters/spin_box.hpp"

namespace gui
{
namespace sa
{
class RcInputWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = RcInputWidget;
  using super = BaseSettingWidget;

public:
  explicit RcInputWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  int numOfSbusChannels() const;

private:
  ParamGetterWidget_SpinBox* num_sbus_channels_;
};
};  // namespace sa
}  // namespace gui
