#pragma once

#include <memory>

#include "../utils/urdf_clone.hpp"
#include "./geometry_view_model.hpp"
#include "./material_view_model.hpp"
#include "./collision_view_model.hpp"
#include "./inertial_view_model.hpp"
#include "./visual_view_model.hpp"
#include "./joint_view_model.hpp"

namespace urdf_builder
{
namespace view_model
{
class LinkViewModel;

using LinkViewModelPtr = std::shared_ptr<LinkViewModel>;
using V_LinkViewModelPtr = std::vector<LinkViewModelPtr>;

class LinkViewModel : public BaseViewModel<urdf::Link, LinkViewModel>
{
public:
  explicit LinkViewModel(const urdf::LinkSharedPtr& model)
    : BaseViewModel<urdf::Link, LinkViewModel>(model),
      inertial_(std::make_shared<InertialViewModel>(model_->inertial)),
      joint_(std::make_shared<JointViewModel>(model_->parent_joint))
  {
    for (const auto& visual : model_->visual_array)
      visuals_.emplace_back(new VisualViewModel(visual));

    for (const auto& collision : model_->collision_array)
      collisions_.emplace_back(new CollisionViewModel(collision));

    // コンストラクタの時点でURDFと同期しておく．
    // そうしないとMeshをクローンしたときにパスエラーが出る．
    sync();
  }

  QString name() const
  {
    return QString::fromStdString(model_->name);
  }

  void name(const QString& name)
  {
    model_->name = name.toStdString();
  }

  const InertialViewModelPtr& inertial() const
  {
    return inertial_;
  }

  const V_VisualViewModelPtr& visuals() const
  {
    return visuals_;
  }

  const V_CollisionViewModelPtr& collisions() const
  {
    return collisions_;
  }

  const JointViewModelPtr& joint() const
  {
    return joint_;
  }

  void usedLinkNames(const QStringList& used_link_names)
  {
    joint_->usedLinkNames(used_link_names);
  }

  bool isValid() const
  {
    if (model_->name.empty())
      return false;

    if (joint_->usedLinkNames().contains(QString::fromStdString(model_->name)))
      return false;

    return true;
  }

  V_LinkViewModelPtr children() const
  {
    V_LinkViewModelPtr result;
    for (const auto& child : model_->child_links)
      result.emplace_back(new LinkViewModel(child));
    return result;
  }

  /* View Modelの内容をurdf::Linkに反映させる． */
  void sync() override
  {
    inertial_->sync();
    joint_->sync();
    for (const auto& visual : visuals_)
      visual->sync();
    for (const auto& collision : collisions_)
      collision->sync();

    // inertial
    model_->inertial = inertial_->model();

    // collision, collision_array
    model_->collision_array.clear();
    std::transform(
      collisions_.begin(), collisions_.end(), std::back_inserter(model_->collision_array),
      [](const CollisionViewModelPtr& vm) { return vm->model(); });
    if (model_->collision_array.empty())
      model_->collision = nullptr;
    else
      model_->collision = model_->collision_array.front();

    // visual, visual_array
    model_->visual_array.clear();
    std::transform(
      visuals_.begin(), visuals_.end(), std::back_inserter(model_->visual_array),
      [](const VisualViewModelPtr& vm) { return vm->model(); });
    if (model_->visual_array.empty())
      model_->visual = nullptr;
    else
      model_->visual = model_->visual_array.front();

    // parent_joint
    model_->parent_joint = joint_->model();

    // child_joints, child_linksは可視化に影響しないため省略？
  }

  void add(const VisualViewModelPtr& visual)
  {
    visuals_.push_back(visual);

    sync();
  }

  void remove(const VisualViewModelPtr& visual)
  {
    visuals_.erase(std::remove(visuals_.begin(), visuals_.end(), visual), visuals_.end());

    sync();
  }

  void add(const CollisionViewModelPtr& collision)
  {
    collisions_.push_back(collision);

    sync();
  }

  void remove(const CollisionViewModelPtr& collision)
  {
    collisions_.erase(
      std::remove(collisions_.begin(), collisions_.end(), collision), collisions_.end());

    sync();
  }

private:
  InertialViewModelPtr inertial_;
  JointViewModelPtr joint_;
  V_VisualViewModelPtr visuals_;
  V_CollisionViewModelPtr collisions_;
};
}  // namespace view_model
}  // namespace urdf_builder
