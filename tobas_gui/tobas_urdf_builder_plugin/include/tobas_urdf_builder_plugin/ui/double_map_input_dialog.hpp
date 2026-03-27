#pragma once

#include <map>

#include <QDialog>
#include <QDoubleSpinBox>

namespace tobas
{
namespace gui
{
namespace ub
{
namespace ui
{
class DoubleMapInputDialog : public QDialog
{
  Q_OBJECT

  using self = DoubleMapInputDialog;
  using super = QDialog;

  static constexpr double kMaxValue = 1000.;
  static constexpr double kMinValue = 0.;
  static constexpr double kDefaultValue = kMinValue;
  static constexpr double kSingleStep = 0.1;
  static constexpr int kDecimals = 6;

public:
  explicit DoubleMapInputDialog(QWidget* parent, const QString& title, const QStringList& field_names);

  const double& getValue(const QString& field) const;

private Q_SLOTS:
  void onSpinBoxValueChanged(double value);

private:
  std::map<QString, double> field2value_;
  std::map<QDoubleSpinBox*, QString> spinbox2field_;
};
}  // namespace ui
}  // namespace ub
}  // namespace gui
}  // namespace tobas
