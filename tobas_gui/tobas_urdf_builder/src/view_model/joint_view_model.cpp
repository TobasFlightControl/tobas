#include "../../include/tobas_urdf_builder/view_model/joint_view_model.hpp"
#include "../../include/tobas_urdf_builder/utils/urdf_clone.hpp"

using namespace std;

using JointType = decltype(urdf::Joint::type);

static const map<JointType, QString> kJointTypeToNameMap = {
  { urdf::Joint::FIXED, "Fixed" },           { urdf::Joint::REVOLUTE, "Revolute" },
  { urdf::Joint::CONTINUOUS, "Continuous" }, { urdf::Joint::PRISMATIC, "Prismatic" },
  { urdf::Joint::FLOATING, "Floating" },     { urdf::Joint::PLANAR, "Planar" },
};

static const map<QString, JointType> kJointNameToTypeMap = {
  { "Fixed", urdf::Joint::FIXED },           { "Revolute", urdf::Joint::REVOLUTE },
  { "Continuous", urdf::Joint::CONTINUOUS }, { "Prismatic", urdf::Joint::PRISMATIC },
  { "Floating", urdf::Joint::FLOATING },     { "Planar", urdf::Joint::PLANAR },
};

namespace urdf_builder
{
namespace view_model
{
JointViewModel::JointViewModel(const urdf::JointSharedPtr& model) : BaseViewModel<urdf::Joint, JointViewModel>(model)
{
  if (model_->type == urdf::Joint::UNKNOWN)
    model_->type = urdf::Joint::FIXED;

  if (limitsEnabled())
    limits_.reset(new JointLimitsViewModel(model_->limits));
}

void JointViewModel::sync()
{
  if (limitsEnabled())
    model_->limits = limits_->model();
}

QString JointViewModel::name() const
{
  return QString::fromStdString(model_->name);
}

void JointViewModel::name(const QString& name)
{
  model_->name = name.toStdString();
}

const QString& JointViewModel::type() const
{
  return kJointTypeToNameMap.at(model_->type);
}

void JointViewModel::type(const QString& type)
{
  model_->type = kJointNameToTypeMap.at(type);

  if (limitsEnabled())
    limits_.reset(new JointLimitsViewModel(model_->limits));
  else
    limits_ = nullptr;
}

const urdf::Pose& JointViewModel::origin() const
{
  return model_->parent_to_joint_origin_transform;
}

void JointViewModel::origin(const urdf::Pose& origin)
{
  model_->parent_to_joint_origin_transform = origin;
}

QString JointViewModel::parentLinkName() const
{
  return QString::fromStdString(model_->parent_link_name);
}

void JointViewModel::parentLinkName(const QString& name)
{
  model_->parent_link_name = name.toStdString();
}

QString JointViewModel::childLinkName() const
{
  return QString::fromStdString(model_->child_link_name);
}

void JointViewModel::childLinkName(const QString& name)
{
  model_->child_link_name = name.toStdString();
}

const urdf::Vector3& JointViewModel::axis() const
{
  return model_->axis;
}

void JointViewModel::axis(const urdf::Vector3& axis)
{
  model_->axis = axis;
}

const JointLimitsViewModelPtr& JointViewModel::limits()
{
  return limits_;
}

bool JointViewModel::limitsEnabled() const
{
  return model_->type == urdf::Joint::REVOLUTE || model_->type == urdf::Joint::PRISMATIC;
}

bool JointViewModel::isFixed() const
{
  return model_->type == urdf::Joint::FIXED;
}
}  // namespace view_model
}  // namespace urdf_builder
