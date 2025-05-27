#pragma once

#include <eigen3/Eigen/Core>

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace sa
{
class ParamGetterWidget_DoublePair : public ParamGetterWidget<std::pair<double, double>>
{
  Q_OBJECT

  using self = ParamGetterWidget_DoublePair;
  using super = ParamGetterWidget<std::pair<double, double>>;

Q_SIGNALS:
  void valueChanged(std::pair<double, double> value);

public:
  explicit ParamGetterWidget_DoublePair(const QString& param_name, const QString& description_text);

  std::pair<double, double> getValue() const override;
  bool setValue(const std::pair<double, double>& src) override;

  void setDecimals(int decimals);
  void setMinimum(double minimum);
  void setMaximum(double maximum);
  void setSingleStep(double single_step);
  void setSuffix(const QString& suffix);

  double first() const;
  double second() const;

private Q_SLOTS:
  void onValueChanged(double value);

private:
  qt::DoubleSpinBox* first_;
  qt::DoubleSpinBox* second_;
};
}  // namespace sa
}  // namespace gui
