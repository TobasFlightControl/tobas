#pragma once

#include <memory>

#include <QtCore/QtCore>

#include "../utils/urdf_clone.hpp"
#include "./base_view_model.hpp"
#include "./joint_limits_view_model.hpp"

namespace gui
{
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
  explicit JointViewModel(const urdf::JointSharedPtr& model);

  void sync() override;

  QString name() const;
  void name(const QString& name);

  const QString& type() const;
  void type(const QString& type);

  const urdf::Pose& origin() const;
  void origin(const urdf::Pose& origin);

  QString parentLinkName() const;
  void parentLinkName(const QString& name);

  QString childLinkName() const;
  void childLinkName(const QString& name);

  const urdf::Vector3& axis() const;
  void axis(const urdf::Vector3& axis);

  const JointLimitsViewModelPtr& limits();

  bool limitsEnabled() const;
  bool isFixed() const;

private:
  JointLimitsViewModelPtr limits_;
};
}  // namespace view_model
}  // namespace urdf_builder
}  // namespace gui
