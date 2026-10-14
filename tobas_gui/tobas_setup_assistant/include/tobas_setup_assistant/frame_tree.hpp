// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QTreeWidget>

#include <tobas_kdl/tree.hpp>

#include "./rviz.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class FrameTreeWidget : public QTreeWidget
{
  Q_OBJECT

  using self = FrameTreeWidget;

public:
  explicit FrameTreeWidget(const kdl::Tree& _tree, RvizWidget* _rviz);

  void updateInternalDataStructures();

private Q_SLOTS:
  void onItemChanged(QTreeWidgetItem* _ite);

  /** Adjust column widths according to string length. */
  void resizeColumns();

private:
  const kdl::Tree& tree_;
  RvizWidget* const rviz_;

  void addTreeItemsRec(QTreeWidgetItem* _parent_item);
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
