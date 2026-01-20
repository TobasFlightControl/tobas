#pragma once

#include <QLineEdit>

#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "../param_getters/combo_box.hpp"
#include "./base_setting.hpp"

namespace gui
{
namespace sa
{
class NetworkWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = NetworkWidget;
  using super = BaseSettingWidget;

  static constexpr int kAutoIdx = 0;
  static constexpr int kWiredIdx = 1;
  static constexpr int kWirelessIdx = 2;
  static constexpr int kAccessPointIdx = 3;
  static constexpr int kOtherIdx = 4;

  static constexpr char kNifNameLabel[] = "Network Interface Name";

public:
  explicit NetworkWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  QString networkInterfaceName() const;

private:
  ParamGetterWidget_ComboBox* nif_type_;

  QLabel* nif_name_label_;
  QLineEdit* nif_name_;

private Q_SLOTS:
  void onNifTypeChanged(int index);
};
};  // namespace sa
}  // namespace gui
