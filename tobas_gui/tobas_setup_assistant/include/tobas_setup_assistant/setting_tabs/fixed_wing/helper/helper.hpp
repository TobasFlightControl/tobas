#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_kdl/tree.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_uadf/model.hpp>

#include "../coefs/coefs.hpp"
#include "../vehicle.hpp"
#include "./calculator.hpp"
#include "./control_surfaces.hpp"
#include "./wings.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
namespace hp
{
class HelperWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = HelperWidget;
  using super = qt::ScrollArea;

  static constexpr char kWingsLabel[] = "Wings";
  static constexpr char kControlSurfacesLabel[] = "Control Surfaces";

public:
  explicit HelperWidget(
    const uadf::Model& uadf,
    const kdl::Tree& tree,
    cf::CoefficientsWidget* coefs);

  const char* name() const;
  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

private:
  WingsWidget* wings_;
  ControlSurfacesWidget* control_surfaces_;
  Calculator* calculator_;
};
}  // namespace hp
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
