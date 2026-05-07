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
  using super = QTreeWidget;

public:
  explicit FrameTreeWidget(const kdl::Tree& tree, RvizWidget* rviz);

  void updateInternalDataStructures();

private Q_SLOTS:
  void onItemClicked(QTreeWidgetItem* item, int col);

  /* 文字列の長さに応じて列の幅を調整する． */
  void resizeColumns();

private:
  const kdl::Tree& tree_;
  RvizWidget* const rviz_;

  void addTreeItemsRec(QTreeWidgetItem* parent_item);
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
