#include "tobas_urdf_builder_plugin/view_model/link_view_model.hpp"

#include "tobas_urdf_builder_plugin/utils/urdf_clone.hpp"

using namespace std;

namespace gui
{
namespace urdf_builder
{
namespace view_model
{
LinkViewModel::LinkViewModel(const urdf::LinkSharedPtr& model)
  : BaseViewModel<urdf::Link, LinkViewModel>(model)
  , inertial_(make_shared<InertialViewModel>(model_->inertial))
  , joint_(make_shared<JointViewModel>(model_->parent_joint))
{
  for (const auto& visual : model_->visual_array) {
    visuals_.emplace_back(new VisualViewModel(visual));
  }

  for (const auto& collision : model_->collision_array) {
    collisions_.emplace_back(new CollisionViewModel(collision));
  }

  // コンストラクタの時点でURDFと同期しておく．
  // そうしないとMeshをクローンしたときにパスエラーが出る．
  sync();
}

void LinkViewModel::sync()
{
  inertial_->sync();
  joint_->sync();
  for (const auto& visual : visuals_) {
    visual->sync();
  }
  for (const auto& collision : collisions_) {
    collision->sync();
  }

  // inertial
  model_->inertial = inertial_->model();

  // collision, collision_array
  model_->collision_array.clear();
  transform(
    collisions_.begin(),
    collisions_.end(),
    back_inserter(model_->collision_array),
    [](const CollisionViewModelPtr& cvm) { return cvm->model(); });
  if (model_->collision_array.empty()) {
    model_->collision.reset();
  }
  else {
    model_->collision = model_->collision_array.front();
  }

  // visual, visual_array
  model_->visual_array.clear();
  transform(
    visuals_.begin(),
    visuals_.end(),
    back_inserter(model_->visual_array),
    [](const VisualViewModelPtr& vvm) { return vvm->model(); });
  if (model_->visual_array.empty()) {
    model_->visual.reset();
  }
  else {
    model_->visual = model_->visual_array.front();
  }

  // parent_joint
  model_->parent_joint = joint_->model();

  // child_joints, child_linksは変化が起きたらその都度更新
}

QString LinkViewModel::name() const
{
  return QString::fromStdString(model_->name);
}

void LinkViewModel::name(const QString& name)
{
  model_->name = name.toStdString();

  // 整合性をとるために上下の関節に含まれるリンク名も変更する
  joint_->childLinkName(name);
  for (const auto& child_link : model_->child_links) {
    child_link->parent_joint->parent_link_name = name.toStdString();
  }
}

const InertialViewModelPtr& LinkViewModel::inertial()
{
  return inertial_;
}

const V_VisualViewModelPtr& LinkViewModel::visuals()
{
  return visuals_;
}

const V_CollisionViewModelPtr& LinkViewModel::collisions()
{
  return collisions_;
}

const JointViewModelPtr& LinkViewModel::joint()
{
  return joint_;
}

V_LinkViewModelPtr LinkViewModel::children() const
{
  V_LinkViewModelPtr result;
  for (const auto& child : model_->child_links) {
    result.emplace_back(new LinkViewModel(child));
  }
  return result;
}

void LinkViewModel::add(const VisualViewModelPtr& visual)
{
  visuals_.push_back(visual);

  sync();
}

void LinkViewModel::remove(const VisualViewModelPtr& visual)
{
  visuals_.erase(std::remove(visuals_.begin(), visuals_.end(), visual), visuals_.end());

  sync();
}

void LinkViewModel::add(const CollisionViewModelPtr& collision)
{
  collisions_.push_back(collision);

  sync();
}

void LinkViewModel::remove(const CollisionViewModelPtr& collision)
{
  collisions_.erase(std::remove(collisions_.begin(), collisions_.end(), collision), collisions_.end());

  sync();
}
}  // namespace view_model
}  // namespace urdf_builder
}  // namespace gui
