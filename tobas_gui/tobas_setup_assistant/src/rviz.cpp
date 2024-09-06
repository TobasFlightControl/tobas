#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <rviz_common/display_group.hpp>

#include <tobas_std_tools/check.hpp>
#include <tobas_qt_tools/rviz.hpp>

#include "tobas_setup_assistant/rviz.hpp"
#include "tobas_setup_assistant/common.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace setup_assistant
{
RvizWidget::RvizWidget(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
  const RobotInfo& robot)
  : robot_(robot)
{
  // Create Rviz frame widget
  const auto pkg_path = fs::path(ament_index_cpp::get_package_share_directory(kPackageName));
  const auto rviz_config_path = pkg_path / "config/setup_assistant.rviz";
  const auto frame = qt::createRvizFrame(rviz_ros_node, QString::fromStdString(rviz_config_path));

  // Setup robot_model_display
  // rviz::Display Class Reference: https://docs.ros.org/en/diamondback/api/rviz/html/classrviz_1_1Display.html
  manager_ = frame->getManager();
  display_ = manager_->getRootDisplayGroup()->getDisplayAt(kRobotStateDisplayIndex);
  TOBAS_CHECK(display_->getName() == "RobotState");

  // 最初は機能をオフにしておく．さもないとrobot_descriptionが見つからないというエラーが出る．
  display_->setBool(false);

  // 使用するプロパティを取得
  enable_visual_ = qobject_cast<rviz_common::properties::BoolProperty*>(display_->subProp("Visual Enabled"));
  enable_collision_ = qobject_cast<rviz_common::properties::BoolProperty*>(display_->subProp("Collision Enabled"));
  reload_ = qobject_cast<rviz_common::properties::BoolProperty*>(display_->subProp("Reload"));
  highlight_link_ = qobject_cast<rviz_common::properties::StringProperty*>(display_->subProp("Highlight Link"));
  unhighlight_link_ = qobject_cast<rviz_common::properties::StringProperty*>(display_->subProp("Unhighlight Link"));

  enable_visual_->setBool(kDefaultVisualEnabled);
  enable_collision_->setBool(kDefaultCollisionEnabled);

  // 可視化ボタン
  const auto visual_box = new QCheckBox("Show Visual Geometry");
  visual_box->setChecked(kDefaultVisualEnabled);
  const auto collision_box = new QCheckBox("Show Collision Geometry");
  collision_box->setChecked(kDefaultCollisionEnabled);

  // レイアウト
  const auto rows = new QVBoxLayout(this);
  const auto cols = new QHBoxLayout();
  rows->addWidget(frame);
  rows->addLayout(cols);
  cols->addStretch();
  cols->addWidget(visual_box);
  cols->addWidget(collision_box);

  // Connections
  connect(visual_box, &QCheckBox::toggled, this, &self::onVisualBoxToggled);
  connect(collision_box, &QCheckBox::toggled, this, &self::onCollisionBoxToggled);
}

void RvizWidget::onRobotLoaded()
{
  // 有効化
  display_->setBool(true);

  // 固定フレームをルートリンクに設定
  const auto& root_name = robot_.tree().getRootName();
  manager_->setFixedFrame(QString::fromStdString(root_name));

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

void RvizWidget::onVisualBoxToggled(bool checked)
{
  enable_visual_->setBool(checked);
}

void RvizWidget::onCollisionBoxToggled(bool checked)
{
  enable_collision_->setBool(checked);
}
}  // namespace setup_assistant
}  // namespace gui
