#include <tobas_std_tools/check.hpp>

#include "tobas_setup_assistant/setup_assistant.hpp"

namespace gui
{
namespace setup_assistant
{
FrameTreeWidget::FrameTreeWidget(SetupAssistant* main) : main_(main)
{
  setColumnCount(1);
  setHeaderLabels({ "Frames Tree" });

  setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  connect(this, &self::itemClicked, this, &self::onItemClicked);
  connect(this, &self::itemExpanded, this, &self::resizeColumns);
  connect(this, &self::itemCollapsed, this, &self::resizeColumns);
}

void FrameTreeWidget::updateInternalDataStructures()
{
  // ツリーを消去
  clear();

  // ルートリンクから再帰的にリンクをTreeに追加していく．
  // cf. https://doc.qt.io/qtforpython/tutorials/basictutorial/treewidget.html
  const auto& root_name = main_->tree().getRootName();
  auto root_item = new QTreeWidgetItem({ QString::fromStdString(root_name) });
  addTreeItemsRec(root_item);
  insertTopLevelItem(0, root_item);

  resizeColumns();
}

void FrameTreeWidget::onItemClicked(QTreeWidgetItem* item, int col)
{
  TOBAS_CHECK(col == 0);
  const auto link_name = item->text(col);
  main_->rviz()->heightLink(link_name);
}

void FrameTreeWidget::resizeColumns()
{
  resizeColumnToContents(0);
}

void FrameTreeWidget::addTreeItemsRec(QTreeWidgetItem* parent_item)
{
  const auto parent_name = parent_item->text(0).toStdString();
  const auto& tree = main_->tree();

  if (tree.isEndSegment(parent_name))
    return;

  const auto parent_it = tree.getSegment(parent_name);
  for (const auto& child_it : parent_it->second.children)
  {
    const auto& child_name = child_it->first;
    auto child_item = new QTreeWidgetItem({ QString::fromStdString(child_name) });
    parent_item->addChild(child_item);
    addTreeItemsRec(child_item);
  }
}
}  // namespace setup_assistant
}  // namespace gui
