#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <rviz_common/display_group.hpp>

#include <tobas_std_tools/check.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/rviz.hpp>

#include "tobas_setup_assistant/setup_assistant.hpp"

namespace gui
{
namespace setup_assistant
{
RvizWidget::RvizWidget(SetupAssistant* main) : main_(main)
{
  // Create Rviz frame widget
  const auto pkg_path = ament_index_cpp::get_package_share_directory(kPackageName);
  const auto rviz_config_path = path::join(pkg_path, "config/setup_assistant.rviz");
  auto frame = qt::createRvizFrame(main->rvizRosNode(), QString::fromStdString(rviz_config_path));

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
  auto visual_box = new QCheckBox("Show Visual Geometry");
  visual_box->setChecked(kDefaultVisualEnabled);
  auto collision_box = new QCheckBox("Show Collision Geometry");
  collision_box->setChecked(kDefaultCollisionEnabled);

  // レイアウト
  auto rows = new QVBoxLayout();
  auto cols = new QHBoxLayout();
  setLayout(rows);
  rows->addWidget(frame);
  rows->addLayout(cols);
  cols->addStretch();
  cols->addWidget(visual_box);
  cols->addWidget(collision_box);
  setMinimumWidth(kMinWidth);

  // Connections
  connect(visual_box, &QCheckBox::toggled, this, &self::onVisualBoxToggled);
  connect(collision_box, &QCheckBox::toggled, this, &self::onCollisionBoxToggled);
}

void RvizWidget::updateInternalDataStructures()
{
  // 有効化
  display_->setBool(true);

  // 固定フレームをルートリンクに設定
  const auto& root_name = main_->tree().getRootName();
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
