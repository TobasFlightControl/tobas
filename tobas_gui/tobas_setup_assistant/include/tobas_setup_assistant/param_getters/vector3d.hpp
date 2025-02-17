#pragma once

#include <eigen3/Eigen/Core>

#include "./base.hpp"
#include "./scalar_getter.hpp"

namespace gui
{
namespace sa
{
class ParamGetterWidget_Vector3d : public ParamGetterWidget<Eigen::Vector3d>
{
  Q_OBJECT

  using self = ParamGetterWidget_Vector3d;
  using super = ParamGetterWidget<Eigen::Vector3d>;

Q_SIGNALS:
  void valueChanged(Eigen::Vector3d value);

public:
  explicit ParamGetterWidget_Vector3d(const QString& param_name, const QString& description_text);

  Eigen::Vector3d getValue() const override;
  bool setValue(const Eigen::Vector3d& src) override;

  void setDecimals(int decimals);
  void setMinimum(double minimum);
  void setMaximum(double maximum);
  void setSingleStep(double single_step);
  void setSuffix(const QString& suffix);

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
}  // namespace sa
}  // namespace gui
