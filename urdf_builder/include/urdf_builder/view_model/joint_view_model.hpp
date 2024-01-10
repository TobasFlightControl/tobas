#pragma once

#include <memory>
#include <QStringList>

#include "./base_view_model.hpp"
#include "./joint_limits_view_model.hpp"
#include "../utils/urdf_clone.hpp"

namespace urdf_builder
{
namespace view_model
{
class JointViewModel;

using JointViewModelPtr = std::shared_ptr<JointViewModel>;
using V_JointViewModelPtr = std::vector<JointViewModelPtr>;

class JointViewModel : public BaseViewModel<urdf::Joint, JointViewModel>
{
public:
  explicit JointViewModel(const urdf::JointSharedPtr& model)
    : BaseViewModel<urdf::Joint, JointViewModel>(model)
  {
    if (model_->type == urdf::Joint::UNKNOWN)
      model_->type = urdf::Joint::FIXED;

    if (limitsEnabled())
      limits_.reset(new JointLimitsViewModel(model_->limits));
  }

  QString name() const
  {
    return QString::fromStdString(model_->name);
  }

  void name(const QString& name)
  {
    model_->name = name.toStdString();
  }

  const QString& type() const;
  void type(const QString& type);

  const urdf::Pose& origin() const
  {
    return model_->parent_to_joint_origin_transform;
  }

  void origin(const urdf::Pose& origin)
  {
    model_->parent_to_joint_origin_transform = origin;
  }

  QString parentLinkName() const
  {
    return QString::fromStdString(model_->parent_link_name);
  }

  void parentLinkName(const QString& name)
  {
    model_->parent_link_name = name.toStdString();
  }

  QString childLinkName() const
  {
    return QString::fromStdString(model_->child_link_name);
  }

  void childLinkName(const QString& name)
  {
    model_->child_link_name = name.toStdString();
  }

  const QStringList& usedLinkNames() const
  {
    return link_names_;
  }

  void usedLinkNames(const QStringList& links)
  {
    link_names_ = links;
  }

  const urdf::Vector3& axis() const
  {
    return model_->axis;
  }

  void axis(const urdf::Vector3& axis)
  {
    model_->axis = axis;
  }

  bool limitsEnabled() const
  {
    return model_->type == urdf::Joint::REVOLUTE || model_->type == urdf::Joint::PRISMATIC;
  }

  bool isFixed() const
  {
    return model_->type == urdf::Joint::FIXED;
  }

  const JointLimitsViewModelPtr& limits() const
  {
    return limits_;
  }

  void sync() override
  {
    if (limitsEnabled())
      model_->limits = limits_->model();
  }

  void generateName()
  {
    name(childLinkName() + "_to_" + parentLinkName() + "_joint");
  }

private:
  QStringList link_names_;
  JointLimitsViewModelPtr limits_;
};
}  // namespace view_model
}  // namespace urdf_builder
