#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class ParamGetterWidget_SpinBox : public ParamGetterWidget<int>
{
  Q_OBJECT

  using super = ParamGetterWidget<int>;

Q_SIGNALS:
  void valueChanged(int value);

public:
  explicit ParamGetterWidget_SpinBox(
    const QString& param_name,
    const QString& description_text = "",
    int minimum = std::numeric_limits<int>::lowest(),
    int maximum = std::numeric_limits<int>::max(),
    int single_step = 1,
    std::optional<int> _default = std::nullopt,
    const QString& suffix = "");

  int get() const override;
  bool set(const int& src) override;

private Q_SLOTS:
  void onValueChanged(int value);

private:
  qt::SpinBox* spin_box_;
};
}  // namespace setup_assistant
}  // namespace gui
