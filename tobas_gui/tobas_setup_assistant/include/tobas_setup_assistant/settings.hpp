#pragma once

#include <QToolBox>

#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "./setting_tabs/author_information.hpp"
#include "./setting_tabs/controller/controller.hpp"
#include "./setting_tabs/extra_joints.hpp"
#include "./setting_tabs/fixed_wing/fixed_wing.hpp"
#include "./setting_tabs/hardware/hardware.hpp"
#include "./setting_tabs/observer.hpp"
#include "./setting_tabs/pre_arm_check.hpp"
#include "./setting_tabs/propulsion_system/propulsion_system.hpp"
#include "./setting_tabs/rc_input.hpp"
#include "./setting_tabs/remote_connection/remote_connection.hpp"
#include "./setting_tabs/simulation.hpp"

namespace gui
{
namespace sa
{
class SettingsWidget : public QWidget
{
  Q_OBJECT

  using self = SettingsWidget;
  using super = QWidget;

public:
  propulsion::PropulsionSystemWidget* propulsion_system;
  fw::FixedWingWidget* fixed_wing;
  hw::HardwareWidget* hardware;
  rc::RemoteConnectionWidget* remote_connection;
  ctrl::ControllerWidget* controller;
  ObserverWidget* observer;
  RcInputWidget* rc_input;
  ExtraJointsWidget* extra_joints;
  PreArmCheckWidget* pre_arm_check;
  SimulationWidget* simulation;
  AuthorInformationWidget* author_info;

  explicit SettingsWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf, const kdl::Tree& tree, Signals& sig);

  void updateInternalDataStructures();

  bool isValid();

  YAML::Node dump() const;
  bool load(const YAML::Node& node);

  void setFrameType(FrameType type);

private:
  const uadf::Model& uadf_;

  QToolBox* toolbox_;
  qt::StackedWidget* stack_;
  qt::ListWidget* basic_list_;
  qt::ListWidget* additional_list_;

  int getIndex(BaseSettingWidget* page) const;
  void addEntry(QListWidget* list, BaseSettingWidget* page);
  void setCurrentPage(int idx);
  void setCurrentPage(BaseSettingWidget* page);
  void setPageEnabled(int idx, bool enabled);
  void setPageEnabled(BaseSettingWidget* page, bool enabled);
  void setListItemEnabled(QListWidgetItem* item, bool enabled);

private Q_SLOTS:
  void onListItemChanged(QListWidgetItem* item);
};
}  // namespace sa
}  // namespace gui
