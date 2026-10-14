// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/frame_tree.hpp"

namespace tobas::gui::sa
{
FrameTreeWidget::FrameTreeWidget(const kdl::Tree& _tree, RvizWidget* _rviz) : tree_(_tree), rviz_(_rviz)
{
  setColumnCount(1);
  setHeaderLabels({ "Frames Tree" });

  setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  connect(this, &self::currentItemChanged, this, &self::onItemChanged);
  connect(this, &self::itemExpanded, this, &self::resizeColumns);
  connect(this, &self::itemCollapsed, this, &self::resizeColumns);
}

void FrameTreeWidget::updateInternalDataStructures()
{
  // Clear the tree.
  clear();

  // Add links to the tree recursively from the root link.
  // Ref: https://doc.qt.io/qtforpython/tutorials/basictutorial/treewidget.html
  const auto& root_name = tree_.getRootName();
  const auto root_item = new QTreeWidgetItem({ QString::fromStdString(root_name) });
  addTreeItemsRec(root_item);
  insertTopLevelItem(0, root_item);

  resizeColumns();
}

void FrameTreeWidget::onItemChanged(QTreeWidgetItem* _item)
{
  // Clearing the tree during a reload also clears the current item.
  if (!_item) {
    return;
  }

  const auto link_name = _item->text(0);
  rviz_->heightLink(link_name);
}

void FrameTreeWidget::resizeColumns()
{
  resizeColumnToContents(0);
}

void FrameTreeWidget::addTreeItemsRec(QTreeWidgetItem* _parent_item)
{
  const auto parent_name = _parent_item->text(0).toStdString();

  if (tree_.isEndSegment(parent_name)) {
    return;
  }

  const auto parent_it = tree_.getSegment(parent_name);
  for (const auto& child_it : parent_it->second.children) {
    const auto& child_name = child_it->first;
    const auto child_item = new QTreeWidgetItem({ QString::fromStdString(child_name) });
    _parent_item->addChild(child_item);
    addTreeItemsRec(child_item);
  }
}
}  // namespace tobas::gui::sa
