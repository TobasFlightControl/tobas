#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/font.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/propulsion_system.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
PropulsionSystemWidget::PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot)
  : node_(node), robot_(robot)
{
  const auto links_label = new QLabel("Available Links");
  links_label->setFont(qt::DefaultFont(kLabelPSize, QFont::Bold));
  links_label->setAlignment(Qt::AlignLeft);
  addWidget(links_label);

  available_ = new AvailableLinksWidget(robot_);
  connect(available_, &AvailableLinksWidget::linkRemoved, this, &self::onAvailableLinkRemoved);
  addWidget(available_);

  selected_ = new SelectedLinksWidget(node_, robot_);
  connect(selected_, &SelectedLinksWidget::linkRemoved, this, &self::onSelectedLinkRemoved);
  addWidget(selected_);
}

const char* PropulsionSystemWidget::name() const
{
  return "Propulsion";
}

const char* PropulsionSystemWidget::title() const
{
  return "Define Propulsion System";
}

const char* PropulsionSystemWidget::description() const
{
  return "Configure the propulsion system. "
         "Please add the link you intend to use for the propulsion system from the Available Links, "
         "and input the necessary information for each.";
}

void PropulsionSystemWidget::onOpened()
{
}

void PropulsionSystemWidget::updateInternalDataStructures()
{
  available_->updateInternalDataStructures();
  selected_->updateInternalDataStructures();
}

bool PropulsionSystemWidget::isValid()
{
  if (!available_->isValid())
    return false;
  if (!selected_->isValid())
    return false;

  return true;
}

YAML::Node PropulsionSystemWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < selected_->count(); ++i)
  {
    const auto link_name = selected_->linkName(i);
    node[link_name.toStdString()] = selected_->widget(i)->dump();
  }

  return node;
}

void PropulsionSystemWidget::load(const YAML::Node& node)
{
  for (const auto& pair : node)
  {
    const auto link_name = pair.first.as<QString>();
    const auto& sub_node = pair.second;

    // リンクをAvailableからSelectedに移動させる
    available_->remove(link_name);
    selected_->add(link_name);

    // 選択リンクの設定を更新
    selected_->widget(link_name)->load(sub_node);
  }
}

const AvailableLinksWidget* PropulsionSystemWidget::available() const
{
  return available_;
}

const SelectedLinksWidget* PropulsionSystemWidget::selected() const
{
  return selected_;
}

void PropulsionSystemWidget::onAvailableLinkRemoved(const QString& link_name)
{
  selected_->add(link_name);
}

void PropulsionSystemWidget::onSelectedLinkRemoved(const QString& link_name)
{
  available_->add(link_name);
}
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
