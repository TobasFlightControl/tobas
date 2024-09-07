#include <QLabel>
#include <QVBoxLayout>

#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/servo_joint/servo_joint.hpp"
#include "tobas_setup_assistant/common.hpp"

namespace gui
{
namespace setup_assistant
{
namespace servo_joint
{
ServoJointsWidget::ServoJointsWidget(const RobotInfo& robot) : robot_(robot)
{
}

const char* ServoJointsWidget::name() const
{
  return "Custom Joints";
}

const char* ServoJointsWidget::title() const
{
  return "Define Custom Joints";
}

const char* ServoJointsWidget::description() const
{
  return "Configure the settings for joints with transmissions "
         "other than those in the propulsion system and fixed-wing control surfaces.";
}

void ServoJointsWidget::onInit()
{
  const auto available_links_label = new QLabel("Available Links");
  available_links_label->setFont(qt::DefaultFont(kLabelPSize, QFont::Bold));
  available_links_label->setAlignment(Qt::AlignLeft);
  addWidget(available_links_label);

  available_ = new AvailableLinksWidget(robot_);
  selected_ = new SelectedLinksWidget(robot_);
  add_remove_ = new AddRemoveButtonsWidget(available_, selected_);

  addWidget(available_);
  addWidget(add_remove_);
  addWidget(selected_);
}

void ServoJointsWidget::onOpened()
{
}

void ServoJointsWidget::updateInternalDataStructures()
{
  available_->updateInternalDataStructures();
  selected_->updateInternalDataStructures();
}

bool ServoJointsWidget::isValid()
{
  for (int row = 0; row < selected_->rowCount(); ++row)
  {
    const auto jnt_name = selected_->jointName(row);
    const auto home_pos = selected_->homePosition(row);
    const auto min_pos = selected_->minPosition(row);
    const auto max_pos = selected_->maxPosition(row);

    if (min_pos > max_pos)
    {
      qt::qErrorBox(this, "Position limit of joint \"" + jnt_name + "\" is invalid.");
      return false;
    }
    if (home_pos < min_pos || max_pos < home_pos)
    {
      qt::qErrorBox(this, "Home position of joint \"" + jnt_name + "\" is out of its limit.");
      return false;
    }
  }

  return true;
}

YAML::Node ServoJointsWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  for (int row = 0; row < selected_->rowCount(); ++row)
  {
    const auto link_name = selected_->linkName(row);
    node[link_name.toStdString()] = selected_->dump(link_name);
  }

  return node;
}

void ServoJointsWidget::load(const YAML::Node& node)
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

const AvailableLinksWidget* ServoJointsWidget::available() const
{
  return available_;
}

const SelectedLinksWidget* ServoJointsWidget::selected() const
{
  return selected_;
}
}  // namespace servo_joint
}  // namespace setup_assistant
}  // namespace gui
