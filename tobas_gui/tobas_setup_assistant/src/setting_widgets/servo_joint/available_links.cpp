#include <QDebug>

#include "tobas_setup_assistant/setting_tabs/servo_joint/available_links.hpp"

namespace gui
{
namespace setup_assistant
{
namespace servo_joint
{
AvailableLinksWidget::AvailableLinksWidget(const RobotInfo& robot) : robot_(robot)
{
}

void AvailableLinksWidget::updateInternalDataStructures()
{
  clear();

  for (const auto& [link_name, elem] : robot_.tree().getSegments())
  {
    if (link_name == robot_.tree().getRootName())
      continue;

    const auto& joint = elem.segment.joint();

    // 可動関節をもつことを確認
    if (joint.type == kdl::Joint::Fixed)
      continue;

    // リミットが正しく設定されていることを確認
    if (joint.upper_limit < joint.lower_limit)
      continue;
    if (joint.max_velocity < 0.)
      continue;
    if (joint.max_effort < 0.)
      continue;

    // リンク名をリストに追加
    add(QString::fromStdString(link_name));
  }

  sortItems();
}

QString AvailableLinksWidget::selected() const
{
  const auto cur_item = currentItem();
  if (cur_item == nullptr)
    return "";
  else
    return cur_item->text();
}

void AvailableLinksWidget::add(const QString& link_name)
{
  if (contains(link_name))
  {
    qWarning() << link_name << " already exists in the list of available links.";
    return;
  }

  addItem(link_name);
  sortItems();
}

void AvailableLinksWidget::remove(const QString& link_name)
{
  const auto items = findItems(link_name, Qt::MatchExactly);

  if (items.size() == 0)
  {
    qWarning() << link_name << " does not exist in the list of available links.";
    return;
  }

  takeItem(row(items[0]));
}
}  // namespace servo_joint
}  // namespace setup_assistant
}  // namespace gui
