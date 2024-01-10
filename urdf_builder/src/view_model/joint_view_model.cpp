#include "../../include/urdf_builder/view_model/joint_view_model.hpp"
#include "../../include/urdf_builder/utils/urdf_clone.hpp"

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
}  // namespace view_model
}  // namespace urdf_builder
