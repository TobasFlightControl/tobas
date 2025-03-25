#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/font.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/propulsion_units.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
PropulsionUnitsWidget::PropulsionUnitsWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot, Signals& _signals)
  : signals_(_signals)
{
  const auto available_links_label = new QLabel("Available Links");
  available_links_label->setFont(qt::DefaultFont(kLabelPSize, QFont::Bold));
  available_links_label->setAlignment(Qt::AlignLeft);

  available_ = new AvailableLinksWidget(robot);
  selected_ = new SelectedLinksWidget(node, robot, _signals);

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(available_links_label);
  rows->addWidget(available_);
  rows->addWidget(selected_);
  setLayout(rows);

  // Connection
  connect(available_, &AvailableLinksWidget::linkRemoved, this, &self::onAvailableLinkRemoved);
  connect(selected_, &SelectedLinksWidget::linkRemoved, this, &self::onSelectedLinkRemoved);
}

void PropulsionUnitsWidget::clear()
{
  while (selected_->numUnits() > 0)
    selected_->removeLink(selected_->linkName(0));
}

void PropulsionUnitsWidget::updateInternalDataStructures()
{
  available_->updateInternalDataStructures();
  selected_->updateInternalDataStructures();
}

bool PropulsionUnitsWidget::isValid()
{
  if (!available_->isValid())
    return false;
  if (!selected_->isValid())
    return false;

  return true;
}

YAML::Node PropulsionUnitsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < selected_->count(); ++i)
  {
    const auto link_name = selected_->linkName(i);
    node[link_name.toStdString()] = selected_->widget(i)->dump();
  }

  return node;
}

void PropulsionUnitsWidget::load(const YAML::Node& node)
{
  for (const auto& pair : node)
  {
    const auto link_name = pair.first.as<QString>();
    const auto& sub_node = pair.second;

    // リンクをAvailableからSelectedに移動させる
    available_->removeLink(link_name);

    // 選択リンクの設定を更新
    selected_->widget(link_name)->load(sub_node);
  }
}

const AvailableLinksWidget* PropulsionUnitsWidget::available() const
{
  return available_;
}

const SelectedLinksWidget* PropulsionUnitsWidget::selected() const
{
  return selected_;
}

void PropulsionUnitsWidget::onAvailableLinkRemoved(const QString& link_name)
{
  selected_->addLink(link_name);
  Q_EMIT signals_.rotorLinkAdded(link_name);
}

void PropulsionUnitsWidget::onSelectedLinkRemoved(const QString& link_name)
{
  available_->addLink(link_name);
  available_->sortItems();
  Q_EMIT signals_.rotorLinkRemoved(link_name);
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
