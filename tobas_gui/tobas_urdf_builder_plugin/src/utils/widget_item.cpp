// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/utils/widget_item.hpp"

namespace tobas
{
namespace gui
{
namespace ub
{
namespace ui
{
LinkTreeWidgetItem::LinkTreeWidgetItem(view_model::LinkViewModelPtr _vm, QTreeWidget* _tree_widget)
  : QTreeWidgetItem(_tree_widget, QTreeWidgetItem::Type), vm_(std::move(_vm))
{
}

const view_model::LinkViewModelPtr& LinkTreeWidgetItem::viewModel() const
{
  return vm_;
}

VisualListWidgetItem::VisualListWidgetItem(const view_model::VisualViewModelPtr& _vm)
  : QListWidgetItem(_vm->name()), vm_(_vm)
{
}

const view_model::VisualViewModelPtr& VisualListWidgetItem::viewModel() const
{
  return vm_;
}

CollisionListWidgetItem::CollisionListWidgetItem(const view_model::CollisionViewModelPtr& _vm)
  : QListWidgetItem(_vm->name()), vm_(_vm)
{
}

const view_model::CollisionViewModelPtr& CollisionListWidgetItem::viewModel() const
{
  return vm_;
}
}  // namespace ui
}  // namespace ub
}  // namespace gui
}  // namespace tobas
