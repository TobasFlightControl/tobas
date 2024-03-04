#pragma once

#include <ros/ros.h>
#include <QtWidgets>

#include "./apps/start/start.hpp"
#include "./apps/urdf_builder/urdf_builder.hpp"
#include "./apps/setup_assistant/setup_assistant.hpp"
#include "./apps/simulation/simulation.hpp"
#include "./apps/hardware_setup/hardware_setup.hpp"
#include "./apps/mission_planner/mission_planner.hpp"
#include "./apps/control_system/control_system.hpp"

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
};
}  // namespace tobas_gcs
