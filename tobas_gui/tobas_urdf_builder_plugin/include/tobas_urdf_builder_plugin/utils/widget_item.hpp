// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QListWidgetItem>
#include <QTreeWidget>

#include "../view_model/collision_view_model.hpp"
#include "../view_model/link_view_model.hpp"
#include "../view_model/visual_view_model.hpp"

namespace tobas
{
namespace gui
{
namespace ub
{
namespace ui
{
class LinkTreeWidgetItem : public QTreeWidgetItem
{
public:
  explicit LinkTreeWidgetItem(view_model::LinkViewModelPtr vm, QTreeWidget* tree_widget = nullptr);

  const view_model::LinkViewModelPtr& viewModel() const;

private:
  view_model::LinkViewModelPtr vm_;
};

class VisualListWidgetItem : public QListWidgetItem
{
public:
  explicit VisualListWidgetItem(const view_model::VisualViewModelPtr& vm);

  const view_model::VisualViewModelPtr& viewModel() const;

private:
  view_model::VisualViewModelPtr vm_;
};

class CollisionListWidgetItem : public QListWidgetItem
{
public:
  explicit CollisionListWidgetItem(const view_model::CollisionViewModelPtr& vm);

  const view_model::CollisionViewModelPtr& viewModel() const;

private:
  view_model::CollisionViewModelPtr vm_;
};
}  // namespace ui
}  // namespace ub
}  // namespace gui
}  // namespace tobas
