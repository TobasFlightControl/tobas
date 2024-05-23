#pragma once

#include <memory>
#include <QtCore/QtCore>

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
  explicit JointViewModel(const urdf::JointSharedPtr& model);

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

  const QStringList& usedLinkNames() const;
  void usedLinkNames(const QStringList& links);

  const urdf::Vector3& axis() const;
  void axis(const urdf::Vector3& axis);

  const JointLimitsViewModelPtr& limits() const;

  bool limitsEnabled() const;
  bool isFixed() const;
  void sync() override;
  void generateName();

private:
  QStringList link_names_;
  JointLimitsViewModelPtr limits_;
};
}  // namespace view_model
}  // namespace urdf_builder
