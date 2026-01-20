#pragma once

#include <QButtonGroup>
#include <QLineEdit>
#include <QRadioButton>

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

  static constexpr char kNifTypeKey[] = "nif_type";
  static constexpr char kOtherNifNameKey[] = "other_nif_name";

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
  QButtonGroup* nif_btn_group_;
  QLineEdit* other_nif_name_;

  QRadioButton* addNifTypeButton(const QString& text, int id);

private Q_SLOTS:
  void onOtherButtonToggled(bool checked);
};
};  // namespace sa
}  // namespace gui
