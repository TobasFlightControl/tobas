// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/frame_tree.hpp"

#include <tobas_std_tools/check.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
FrameTreeWidget::FrameTreeWidget(const kdl::Tree& tree, RvizWidget* rviz) : tree_(tree), rviz_(rviz)
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
  const auto& root_name = tree_.getRootName();
  const auto root_item = new QTreeWidgetItem({ QString::fromStdString(root_name) });
  addTreeItemsRec(root_item);
  insertTopLevelItem(0, root_item);

  resizeColumns();
}

void FrameTreeWidget::onItemClicked(QTreeWidgetItem* item, int col)
{
  TOBAS_CHECK(col == 0);
  const auto link_name = item->text(col);
  rviz_->heightLink(link_name);
}

void FrameTreeWidget::resizeColumns()
{
  resizeColumnToContents(0);
}

void FrameTreeWidget::addTreeItemsRec(QTreeWidgetItem* parent_item)
{
  const auto parent_name = parent_item->text(0).toStdString();

  if (tree_.isEndSegment(parent_name)) {
    return;
  }

  const auto parent_it = tree_.getSegment(parent_name);
  for (const auto& child_it : parent_it->second.children) {
    const auto& child_name = child_it->first;
    const auto child_item = new QTreeWidgetItem({ QString::fromStdString(child_name) });
    parent_item->addChild(child_item);
    addTreeItemsRec(child_item);
  }
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
