#include "../include/tobas_gcs/main_widget.hpp"
#include "../include/tobas_gcs/common.hpp"

namespace tobas_gcs
{
MainWidget::MainWidget(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh),
    start_(new StartWidget(this, nh_, pnh_)),
    urdf_builder_(new UrdfBuilderWidget(this, nh_, pnh_)),
    setup_assistant_(new SetupAssistantWidget(this, nh_, pnh_)),
    simulation_(new SimulationWidget(this, nh_, pnh_)),
    hardware_setup_(new HardwareSetupWidget(this, nh_, pnh_)),
    mission_planner_(new MissionPlannerWidget(this, nh_, pnh_)),
    control_system_(new ControlSystemWidget(this, nh_, pnh_))
{
  setWindowTitle(kMainTitle);

  auto* rows = new QVBoxLayout();
  setLayout(rows);

  auto* cols = new QHBoxLayout();
  rows->addLayout(cols);

  auto* combo_box = new QComboBox();
  cols->addWidget(combo_box);
  combo_box->addItem(kStartTitle);
  combo_box->addItem(kUrdfBuilderTitle);
  combo_box->addItem(kSetupAssistantTitle);
  combo_box->addItem(kSimulationTitle);
  combo_box->addItem(kHardwareSetupTitle);
  combo_box->addItem(kMissionPlannerTitle);
  combo_box->addItem(kControlSystemTitle);

  auto* apps = new QStackedWidget();
  rows->addWidget(apps);
  apps->addWidget(start_);
  apps->addWidget(urdf_builder_);
  apps->addWidget(setup_assistant_);
  apps->addWidget(simulation_);
  apps->addWidget(hardware_setup_);
  apps->addWidget(mission_planner_);
  apps->addWidget(control_system_);

  QObject::connect(combo_box, SIGNAL(currentIndexChanged(int)), apps, SLOT(setCurrentIndex(int)));
}
}  // namespace tobas_gcs
