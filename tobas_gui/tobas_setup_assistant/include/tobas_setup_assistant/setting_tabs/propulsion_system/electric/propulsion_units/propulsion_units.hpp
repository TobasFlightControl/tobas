#pragma once

#include "tobas_setup_assistant/signals.hpp"
#include "./available_links.hpp"
#include "./selected_links.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
class PropulsionUnitsWidget : public QWidget
{
  Q_OBJECT

  using self = PropulsionUnitsWidget;
  using super = QWidget;

public:
  explicit PropulsionUnitsWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot, Signals& _signals);

  void clear();
  void updateInternalDataStructures();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  const AvailableLinksWidget* available() const;
  const SelectedLinksWidget* selected() const;

private Q_SLOTS:
  void onAvailableLinkRemoved(const QString& link_name);
  void onSelectedLinkRemoved(const QString& link_name);

private:
  Signals& signals_;

  AvailableLinksWidget* available_;
  SelectedLinksWidget* selected_;
};
};  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
