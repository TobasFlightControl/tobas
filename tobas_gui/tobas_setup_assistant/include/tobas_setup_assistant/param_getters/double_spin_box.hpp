#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace sa
{
class ParamGetterWidget_DoubleSpinBox : public ParamGetterWidget<double>
{
  Q_OBJECT

  using self = ParamGetterWidget_DoubleSpinBox;
  using super = ParamGetterWidget<double>;

Q_SIGNALS:
  void valueChanged(double value);

public:
  explicit ParamGetterWidget_DoubleSpinBox(const QString& param_name, const QString& description_text);

  double getValue() const override;
  bool setValue(const double& src) override;

  void setDecimals(int decimals);
  void setMinimum(double minimum);
  void setMaximum(double maximum);
  void setSingleStep(double single_step);
  void setSuffix(const QString& suffix);

private Q_SLOTS:
  void onValueChanged(double value);

private:
  qt::DoubleSpinBox* spin_box_;
};
}  // namespace sa
}  // namespace gui
