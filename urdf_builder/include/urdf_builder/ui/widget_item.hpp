#pragma once

#include <QTreeWidgetItem>
#include <QListWidgetItem>

#include "../view_model/visual_view_model.hpp"
#include "../view_model/collision_view_model.hpp"
#include "../view_model/link_view_model.hpp"

namespace urdf_builder
{
namespace ui
{
class LinkTreeWidgetItem : public QTreeWidgetItem
{
public:
  explicit LinkTreeWidgetItem(view_model::LinkViewModelPtr vm, QTreeWidget* tree_widget = nullptr)
    : QTreeWidgetItem(tree_widget, QTreeWidgetItem::Type), vm_(std::move(vm))
  {
  }

  const view_model::LinkViewModelPtr& viewModel() const
  {
    return vm_;
  }

private:
  view_model::LinkViewModelPtr vm_;
};

class VisualListWidgetItem : public QListWidgetItem
{
public:
  explicit VisualListWidgetItem(const view_model::VisualViewModelPtr& vm)
    : QListWidgetItem(vm->name()), vm_(vm)
  {
  }

  const view_model::VisualViewModelPtr& viewModel() const
  {
    return vm_;
  }

private:
  view_model::VisualViewModelPtr vm_;
};

class CollisionListWidgetItem : public QListWidgetItem
{
public:
  explicit CollisionListWidgetItem(const view_model::CollisionViewModelPtr& vm)
    : QListWidgetItem(vm->name()), vm_(vm)
  {
  }

  const view_model::CollisionViewModelPtr& viewModel() const
  {
    return vm_;
  }

private:
  view_model::CollisionViewModelPtr vm_;
};
}  // namespace ui
}  // namespace urdf_builder
