#pragma once

#include <tobas_drone_core/control_surface.hpp>

#include "../base.hpp"
#include "./available_links.hpp"
#include "./selected_links.hpp"
#include "./add_remove_buttons.hpp"

namespace gui
{
namespace setup_assistant
{
namespace fixed_wing
{
class ControlSurfacesWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

  using self = ControlSurfacesWidget;
  using super = BaseSelectedLinkSettingWidget;

public:
  explicit ControlSurfacesWidget(const RobotInfo& robot);

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  tobas::ControlSurfaces controlSurfaces() const;

private:
  AvailableLinksWidget* available_;
  SelectedLinksWidget* selected_;
  AddRemoveButtonsWidget* add_remove_;
};
}  // namespace fixed_wing
}  // namespace setup_assistant
}  // namespace gui
