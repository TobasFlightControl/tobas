#pragma once

#include <eigen3/Eigen/Core>

#include "./base.hpp"
#include "./scalar_getter.hpp"

namespace gui
{
namespace setup_assistant
{
class ParamGetterWidget_Pose : public ParamGetterWidget<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
{
  Q_OBJECT

  using self = ParamGetterWidget_Pose;
  using super = ParamGetterWidget<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;

  static constexpr double kXYZSingleStep = 0.1;
  static constexpr double kRPYSingleStep = 0.1;

Q_SIGNALS:
  void valueChanged(std::pair<Eigen::Vector3d, Eigen::Vector3d> value);

public:
  explicit ParamGetterWidget_Pose(
    const QString& param_name,
    const QString& description_text = "",
    int decimals = 3,
    double xyz_min = std::numeric_limits<double>::lowest(),
    double xyz_max = std::numeric_limits<double>::max(),
    const Eigen::Vector3d& xyz_default = Eigen::Vector3d::Zero(),
    const QString& xyz_suffix = " m",
    double rpy_min = std::numeric_limits<double>::lowest(),
    double rpy_max = std::numeric_limits<double>::max(),
    const Eigen::Vector3d& rpy_default = Eigen::Vector3d::Zero(),
    const QString& rpy_suffix = " rad");

  std::pair<Eigen::Vector3d, Eigen::Vector3d> get() const override;
  bool set(const std::pair<Eigen::Vector3d, Eigen::Vector3d>& src) override;

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
}  // namespace setup_assistant
}  // namespace gui
