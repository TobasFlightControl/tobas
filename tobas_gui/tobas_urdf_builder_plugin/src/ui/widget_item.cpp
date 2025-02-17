#include "../../include/tobas_urdf_builder_plugin/ui/widget_item.hpp"

namespace gui
{
namespace urdf_builder
{
namespace ui
{
LinkTreeWidgetItem::LinkTreeWidgetItem(view_model::LinkViewModelPtr vm, QTreeWidget* tree_widget)
  : QTreeWidgetItem(tree_widget, QTreeWidgetItem::Type), vm_(std::move(vm))
{
}

const view_model::LinkViewModelPtr& LinkTreeWidgetItem::viewModel() const
{
  return vm_;
}

VisualListWidgetItem::VisualListWidgetItem(const view_model::VisualViewModelPtr& vm)
  : QListWidgetItem(vm->name()), vm_(vm)
{
}

const view_model::VisualViewModelPtr& VisualListWidgetItem::viewModel() const
{
  return vm_;
}

CollisionListWidgetItem::CollisionListWidgetItem(const view_model::CollisionViewModelPtr& vm)
  : QListWidgetItem(vm->name()), vm_(vm)
{
}

const view_model::CollisionViewModelPtr& CollisionListWidgetItem::viewModel() const
{
  return vm_;
}
}  // namespace ui
}  // namespace urdf_builder
}  // namespace gui
