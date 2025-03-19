#include <QDebug>

#include "tobas_setup_assistant/setting_tabs/fixed_wing/control_surface/available_links.hpp"

namespace gui
{
namespace sa
{
namespace fixed_wing
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

    // 回転関節を持つことを確認
    if (joint.type != kdl::Joint::ROTATION)
      continue;

    // リミットが正しく設定されていることを確認
    if (joint.lower_limit >= 0.)
      continue;
    if (joint.upper_limit <= 0.)
      continue;
    if (joint.upper_limit - joint.lower_limit >= 2 * M_PI)
      continue;
    if (joint.max_velocity <= 0.)
      continue;
    if (joint.max_effort <= 0.)
      continue;

    // エンドリンクであることを確認
    if (!robot_.tree().isEndSegment(link_name))
      continue;

    // 親リンクがルートリンクに固定されていることを確認
    const auto& parent_name = elem.parent->first;
    if (!robot_.tree().isFixedToRoot(parent_name))
      continue;

    // リンク名をリストに追加
    add(QString::fromStdString(link_name));
  }

  sortItems();
}

QString AvailableLinksWidget::selected() const
{
  const auto cur_item = currentItem();
  if (!cur_item)
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
}  // namespace fixed_wing
}  // namespace sa
}  // namespace gui
