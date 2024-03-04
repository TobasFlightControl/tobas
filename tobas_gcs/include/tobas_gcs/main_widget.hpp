#pragma once

#include <ros/ros.h>
#include <QtWidgets>

#include "./apps/apps.hpp"
#include "./connection_manager.hpp"
#include "./package_loader.hpp"

namespace tobas_gcs
{
class MainWidget : public QWidget
{
  Q_OBJECT

public:
  explicit MainWidget(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  StartWidget* start_;
  UrdfBuilderWidget* urdf_builder_;
  SetupAssistantWidget* setup_assistant_;
  SimulationWidget* simulation_;
  HardwareSetupWidget* hardware_setup_;
  MissionPlannerWidget* mission_planner_;
  ControlSystemWidget* control_system_;

  ConnectionManager* connection_manager_;
  PackageLoader* package_loader_;
};
}  // namespace tobas_gcs
