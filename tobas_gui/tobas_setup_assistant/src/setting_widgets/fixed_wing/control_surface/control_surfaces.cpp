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
  const auto rows = new QVBoxLayout();
  setLayout(rows);

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

  connect(
    add_remove_, &AddRemoveButtonsWidget::linkAdded, [this](const QString& link_name) { Q_EMIT linkAdded(link_name); });
  connect(
    add_remove_, &AddRemoveButtonsWidget::linkRemoved,
    [this](const QString& link_name) { Q_EMIT linkRemoved(link_name); });
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

const AvailableLinksWidget* ControlSurfacesWidget::available() const
{
  return available_;
}

const SelectedLinksWidget* ControlSurfacesWidget::selected() const
{
  return selected_;
}

int ControlSurfacesWidget::count() const
{
  return selected_->rowCount();
}
}  // namespace fixed_wing
}  // namespace setup_assistant
}  // namespace gui
