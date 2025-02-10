#pragma once

#include <eigen3/Eigen/Core>

#include "./base.hpp"
#include "./scalar_getter.hpp"

namespace gui
{
namespace sa
{
class ParamGetterWidget_Pose : public ParamGetterWidget<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
{
  Q_OBJECT

  using self = ParamGetterWidget_Pose;
  using super = ParamGetterWidget<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;

  static constexpr int kDecimals = 3;
  static constexpr double kSingleStep = 0.1;
  static constexpr double kDefaultValue = 0.;
  static constexpr char kXYZSuffix[] = " m";
  static constexpr char kRPYSuffix[] = " rad";

Q_SIGNALS:
  void valueChanged(std::pair<Eigen::Vector3d, Eigen::Vector3d> value);

public:
  explicit ParamGetterWidget_Pose(const QString& param_name, const QString& description_text);

  std::pair<Eigen::Vector3d, Eigen::Vector3d> getValue() const override;
  bool setValue(const std::pair<Eigen::Vector3d, Eigen::Vector3d>& src) override;

  double x() const;
  double y() const;
  double z() const;
  double roll() const;
  double pitch() const;
  double yaw() const;

private Q_SLOTS:
  void onValueChanged(double value);

private:
  DoubleGetter* x_;
  DoubleGetter* y_;
  DoubleGetter* z_;
  DoubleGetter* roll_;
  DoubleGetter* pitch_;
  DoubleGetter* yaw_;
};
}  // namespace sa
}  // namespace gui
