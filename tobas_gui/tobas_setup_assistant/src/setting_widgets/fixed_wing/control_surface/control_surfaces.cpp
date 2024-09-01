#include <QLabel>
#include <QVBoxLayout>

#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/font.hpp>

#include "tobas_setup_assistant/setting_tabs/fixed_wing/control_surface/control_surfaces.hpp"
#include "tobas_setup_assistant/common.hpp"

namespace gui
{
namespace setup_assistant
{
namespace fixed_wing
{
ControlSurfacesWidget::ControlSurfacesWidget(const RobotInfo& robot)
{
  const auto rows = new QVBoxLayout(this);

  const auto available_links_label = new QLabel("Available Links");
  available_links_label->setFont(qt::DefaultFont(kLabelPSize, QFont::Bold));
  available_links_label->setAlignment(Qt::AlignLeft);
  rows->addWidget(available_links_label);

  available_ = new AvailableLinksWidget(robot);
  selected_ = new SelectedLinksWidget(robot);
  add_remove_ = new AddRemoveButtonsWidget(available_, selected_);

  rows->addWidget(available_);
  rows->addWidget(add_remove_);
  rows->addWidget(selected_);
}

void ControlSurfacesWidget::updateInternalDataStructures()
{
  available_->updateInternalDataStructures();
  selected_->updateInternalDataStructures();
}

bool ControlSurfacesWidget::isValid()
{
  return true;
}

YAML::Node ControlSurfacesWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int row = 0; row < selected_->rowCount(); ++row)
  {
    const auto link_name = selected_->linkName(row);
    node[link_name.toStdString()] = selected_->dump(link_name);
  }

  return node;
}

void ControlSurfacesWidget::load(const YAML::Node& node)
{
  for (const auto& pair : node)
  {
    const auto link_name = pair.first.as<QString>();
    const auto& sub_node = pair.second;

    // リンクをAvailableからSelectedに移動させる
    available_->remove(link_name);
    selected_->add(link_name);

    // 選択リンクの設定を更新
    selected_->load(link_name, sub_node);
  }
}

tobas::ControlSurfaces ControlSurfacesWidget::controlSurfaces() const
{
  tobas::ControlSurfaces res(selected_->rowCount());

  for (int row = 0; row < selected_->rowCount(); ++row)
  {
    res[row].channel = row;
    res[row].joint_name = selected_->jointName(row).toStdString();
    res[row].angle_limit.lower = selected_->minAngle(row);
    res[row].angle_limit.upper = selected_->maxAngle(row);
    res[row].max_angle_rate = selected_->maxAngleRate(row);
    res[row].c_lift_delta = selected_->liftCoef(row);
    res[row].c_drag_abs_delta = selected_->dragCoef(row);  // FIXME: 正負の確認が必要？
    res[row].c_side_delta = selected_->sideCoef(row);
    res[row].c_roll_delta = selected_->rollCoef(row);
    res[row].c_pitch_delta = selected_->pitchCoef(row);
    res[row].c_yaw_delta = selected_->yawCoef(row);
  }

  return res;
}
}  // namespace fixed_wing
}  // namespace setup_assistant
}  // namespace gui
