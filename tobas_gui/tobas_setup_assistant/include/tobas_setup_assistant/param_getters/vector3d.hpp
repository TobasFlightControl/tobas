#pragma once

#include <eigen3/Eigen/Core>

#include "./base.hpp"
#include "./scalar_getter.hpp"

namespace gui
{
namespace setup_assistant
{
class ParamGetterWidget_Vector3d : public ParamGetterWidget<Eigen::Vector3d>
{
  Q_OBJECT

  using super = ParamGetterWidget<Eigen::Vector3d>;

Q_SIGNALS:
  void valueChanged(Eigen::Vector3d value);

public:
  explicit ParamGetterWidget_Vector3d(
    const QString& param_name,
    const QString& description_text = "",
    int decimals = 3,
    double minimum = std::numeric_limits<double>::lowest(),
    double maximum = std::numeric_limits<double>::max(),
    double single_step = 1.,
    const Eigen::Vector3d& _default = Eigen::Vector3d::Zero(),
    const QString& suffix = "");

  Eigen::Vector3d get() const override;
  bool set(const Eigen::Vector3d& src) override;

  double x() const;
  double y() const;
  double z() const;

private Q_SLOTS:
  void onValueChanged(double value);

private:
  DoubleGetter* x_;
  DoubleGetter* y_;
  DoubleGetter* z_;
};
}  // namespace setup_assistant
}  // namespace gui
