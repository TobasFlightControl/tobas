#pragma once

#include "../base_setting.hpp"
#include "./host/host.hpp"

namespace gui
{
namespace sa
{
namespace rc
{
class RemoteConnectionWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = RemoteConnectionWidget;
  using super = BaseSettingWidget;

  static constexpr char kHostLabel[] = "Host";

public:
  explicit RemoteConnectionWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  QString host() const;

private:
  HostWidget* host_;
};
};  // namespace rc
}  // namespace sa
}  // namespace gui
