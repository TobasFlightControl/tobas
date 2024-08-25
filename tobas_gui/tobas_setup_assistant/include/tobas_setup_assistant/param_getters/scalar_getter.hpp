#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

namespace gui
{
namespace setup_assistant
{
class IntGetter : public QWidget
{
  Q_OBJECT

  using super = QWidget;

Q_SIGNALS:
  void valueChanged(int value);

public:
  explicit IntGetter(
    const QString& name,
    int minimum,
    int maximum,
    int single_step,
    std::optional<int> _default,
    const QString& suffix);

  int get() const;
  bool set(const int& value);

private Q_SLOTS:
  void onValueChanged(int value);

private:
  qt::SpinBox* data_;
};

class DoubleGetter : public QWidget
{
  Q_OBJECT

  using super = QWidget;

Q_SIGNALS:
  void valueChanged(double value);

public:
  explicit DoubleGetter(
    const QString& name,
    int decimals,
    double minimum,
    double maximum,
    double single_step,
    std::optional<double> _default,
    const QString& suffix);

  double get() const;
  bool set(const double& value);

private Q_SLOTS:
  void onValueChanged(double value);

private:
  qt::DoubleSpinBox* data_;
};
}  // namespace setup_assistant
}  // namespace gui
