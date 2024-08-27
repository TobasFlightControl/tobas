#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class ParamGetterWidget_DoubleSpinBox : public ParamGetterWidget<double>
{
  Q_OBJECT

  using self = ParamGetterWidget_DoubleSpinBox;
  using super = ParamGetterWidget<double>;

Q_SIGNALS:
  void valueChanged(double value);

public:
  explicit ParamGetterWidget_DoubleSpinBox(
    const QString& param_name,
    const QString& description_text = "",
    int decimals = 12,
    double minimum = std::numeric_limits<double>::lowest(),
    double maximum = std::numeric_limits<double>::max(),
    double single_step = 1.,
    std::optional<double> _default = std::nullopt,
    const QString& suffix = "");

  double get() const override;
  bool set(const double& src) override;

private Q_SLOTS:
  void onValueChanged(double value);

private:
  qt::DoubleSpinBox* spin_box_;
};
}  // namespace setup_assistant
}  // namespace gui
