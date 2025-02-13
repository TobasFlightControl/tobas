#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_ros2_tools/parameter.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_setup_assistant/rviz.hpp"
#include "tobas_setup_assistant/common.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sa
{
RvizWidget::RvizWidget(const RobotInfo& robot) : robot_(robot), rviz_manager_("rviz_robot_state_display")
{
  // Declare rosparams
  ros2::declareParam(rviz_manager_.rawNode(), kRobotDescriptionParam, tobas::kMinimulURDF);
  ros2::declareParam(rviz_manager_.rawNode(), kRobotDescriptionSemanticParam, tobas::kMinimulURDF);  // MoveItが要求

  // Initialize Rviz
  const fs::path pkg_path(ament_index_cpp::get_package_share_directory(kPackageName));
  const auto rviz_config_path = pkg_path / "config/setup_assistant.rviz";
  rviz_manager_.initialize(QString::fromStdString(rviz_config_path));

  // Setup robot_model_display
  display_ = rviz_manager_.getDisplay("RobotState");

  // 使用するプロパティを取得
  enable_visual_ = qobject_cast<rviz_common::properties::BoolProperty*>(display_->subProp("Visual Enabled"));
  enable_collision_ = qobject_cast<rviz_common::properties::BoolProperty*>(display_->subProp("Collision Enabled"));
  enable_inertia_ = qobject_cast<rviz_common::properties::BoolProperty*>(display_->subProp("Inertial Enabled"));
  highlight_link_ = qobject_cast<rviz_common::properties::StringProperty*>(display_->subProp("Highlight Link"));
  unhighlight_link_ = qobject_cast<rviz_common::properties::StringProperty*>(display_->subProp("Unhighlight Link"));
  reload_ = qobject_cast<rviz_common::properties::BoolProperty*>(display_->subProp("Reload"));

  enable_visual_->setBool(kDefaultVisualEnabled);
  enable_collision_->setBool(kDefaultCollisionEnabled);
  enable_inertia_->setBool(kDefaultInertiaEnabled);

  // 可視化ボタン
  const auto visual_box = new QCheckBox("Show Visual");
  visual_box->setChecked(kDefaultVisualEnabled);
  const auto collision_box = new QCheckBox("Show Collision");
  collision_box->setChecked(kDefaultCollisionEnabled);
  const auto inertia_box = new QCheckBox("Show Inertial");
  inertia_box->setChecked(kDefaultInertiaEnabled);

  // レイアウト
  const auto rows = new QVBoxLayout();
  setLayout(rows);
  const auto cols = new QHBoxLayout();
  rows->addWidget(rviz_manager_.widget());
  rows->addLayout(cols);
  cols->addStretch();
  cols->addWidget(visual_box);
  cols->addWidget(collision_box);
  cols->addWidget(inertia_box);

  // Connection
  connect(&robot, &RobotInfo::loaded, this, &self::onRobotLoaded);
  connect(visual_box, &QCheckBox::toggled, this, &self::onVisualBoxToggled);
  connect(collision_box, &QCheckBox::toggled, this, &self::onCollisionBoxToggled);
  connect(inertia_box, &QCheckBox::toggled, this, &self::onInertiaBoxToggled);
}

void RvizWidget::onRobotLoaded()
{
  // 固定フレームをルートリンクに設定
  const auto& root_name = robot_.tree().getRootName();
  rviz_manager_.setFixedFrame(QString::fromStdString(root_name));

  // URDFを更新
  rviz_manager_.rawNode()->set_parameter(rclcpp::Parameter(kRobotDescriptionParam, robot_.urdfText()));
  rviz_manager_.rawNode()->set_parameter(rclcpp::Parameter(kRobotDescriptionSemanticParam, robot_.urdfText()));

  // ロボットモデルをリロード
  reload_->setBool(false);
  reload_->setBool(true);
}

void RvizWidget::heightLink(const QString& link_name)
{
  if (link_name == highlighted_link_)
    return;

  if (!highlighted_link_.isEmpty())
    unheightLink(highlighted_link_);

  highlight_link_->setValue(link_name);
  highlighted_link_ = link_name;
}

void RvizWidget::unheightLink(const QString& link_name)
{
  unhighlight_link_->setValue(link_name);
}

void RvizWidget::resetTime()
{
  rviz_manager_.resetTime();
}

void RvizWidget::onVisualBoxToggled(bool checked)
{
  enable_visual_->setBool(checked);
}

void RvizWidget::onCollisionBoxToggled(bool checked)
{
  enable_collision_->setBool(checked);
}

void RvizWidget::onInertiaBoxToggled(bool checked)
{
  enable_inertia_->setBool(checked);
}
}  // namespace sa
}  // namespace gui
