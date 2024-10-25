#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_linux/errer.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/progress_dialog.hpp>
#include <tobas_gui_common/package.hpp>
#include <tobas_gui_common/ros2_cli.hpp>

#include "tobas_simulation_gui/simulation.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sim
{
SimulationWidget::SimulationWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);

  terminate_button_ = new QPushButton("Terminate");
  terminate_button_->setFixedSize(kButtonWidth, kButtonHeight);

  wind_params_ = new WindParamsWidget(node);

  // Layout
  const auto button_cols = new QHBoxLayout();
  button_cols->addWidget(start_button_);
  button_cols->addWidget(terminate_button_);
  button_cols->addStretch();

  const auto rows = new QVBoxLayout();
  rows->addLayout(button_cols);
  rows->addWidget(wind_params_);
  rows->addStretch();

  setLayout(rows);

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(terminate_button_, &QPushButton::clicked, this, &self::onTerminateButtonClicked);

  reset();
  setEnabled(false);
}

SimulationWidget::~SimulationWidget()
{
  killGazebo();
}

void SimulationWidget::reset()
{
  killGazebo();

  start_button_->setEnabled(true);
  terminate_button_->setEnabled(false);
  wind_params_->setEnabled(false);
}

void SimulationWidget::killGazebo()
{
  if (launch_pid_ < 0)
    return;

  if (kill(launch_pid_, SIGINT) != 0)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to kill child process: " << linux::strError());
    return;
  }

  if (!cmd_executor_.execute("ps aux | grep \"gz sim\" | grep -v grep | awk '{ print \"kill -9\", $2 }' | sh"))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to kill Gazebo: " << cmd_executor_.getOutput());
    return;
  }

  launch_pid_ = -1;
}

bool SimulationWidget::updateTBSPath(const fs::path& tbs_path)
{
  reset();

  if (!drone_.load(common::getTBSDRNPath(tbs_path)))
  {
    qt::qErrorBox(this, "Failed to load drone configurations.");
    return false;
  }

  tbs_path_ = tbs_path;
  setEnabled(true);

  return true;
}

void SimulationWidget::onStartButtonClicked()
{
  qt::ProgressDialog progress("Gazebo Simulation", 3, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // Build Tobas package
  progress.setLabelText("Building Tobas package.");
  if (!pkg_builder_.build(tbs_path_))
  {
    progress.close();
    qt::qErrorBox(this, "Failed to build Tobas package\n\n:" + QString::fromStdString(pkg_builder_.getOutput()));
    reset();
    return;
  }
  progress.progressStep();

  // Launch Gazebo
  progress.setLabelText("Launching Gazebo simulation.");
  const auto config_pkg_name = common::getTBSConfigName(tbs_path_);
  launch_pid_ = common::roslaunch(config_pkg_name, "gazebo.launch.xml");
  if (launch_pid_ < 0)
  {
    progress.close();
    qt::qErrorBox(this, "Failed to launch Gazebo simulation.");
    reset();
    return;
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Gazebo is launched with pid " << launch_pid_ << ".");
  progress.progressStep();

  // Initialize wind parameter manager
  progress.setLabelText("Initializing wind parameter manager.");
  if (!wind_params_->initialize(drone_.name))
  {
    progress.close();
    reset();
    return;
  }

  start_button_->setEnabled(false);
  terminate_button_->setEnabled(true);
  wind_params_->setEnabled(true);

  progress.close();
  qt::qInfoBox(this, "Gazebo simulation has started.");
}

void SimulationWidget::onTerminateButtonClicked()
{
  reset();
  qt::qInfoBox(this, "Gazebo simulation is terminated.");
}
}  // namespace sim
}  // namespace gui
