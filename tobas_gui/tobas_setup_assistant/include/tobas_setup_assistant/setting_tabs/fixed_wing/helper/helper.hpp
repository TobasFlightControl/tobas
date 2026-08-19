#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_uadf/model.hpp>

#include "./wings.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
class HelperWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = HelperWidget;
  using super = qt::ScrollArea;

  static constexpr char kWingsLabel[] = "Wings";
  static constexpr char kControlSurfacesLabel[] = "Control Surfaces";

public:
  explicit HelperWidget(const uadf::Model& uadf);

  const char* name() const;
  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

private:
  WingsWidget* wings_;
};
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
