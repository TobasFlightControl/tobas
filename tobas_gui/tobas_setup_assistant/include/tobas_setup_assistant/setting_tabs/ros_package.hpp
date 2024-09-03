#pragma once

#include <QPushButton>

#include "./base_setting.hpp"
#include "../robot_info.hpp"
#include "../param_getters/line_edit.hpp"
#include "../param_getters/directory_dialog.hpp"

namespace gui
{
namespace setup_assistant
{
class ROSPackageWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = ROSPackageWidget;
  using super = BaseSettingWidget;

  static constexpr int kTextHeight = 50;
  static constexpr int kButtonHeight = 40;
  static constexpr int kButtonWidth = 100;

Q_SIGNALS:
  void generateButtonClicked();

public:
  explicit ROSPackageWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onInit() override;
  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  QString tbsName() const;
  QString tbsPath() const;

private Q_SLOTS:
  void onPathChanged();
  void onGenerateButtonClicked();

private:
  const rclcpp::Node::SharedPtr node_;
  const RobotInfo& robot_;

  ParamGetterWidget_DirDialog* pardir_;
  ParamGetterWidget_LineEdit* tbs_name_;
  QLabel* tbs_path_;
  QPushButton* generate_button_;
};
};  // namespace setup_assistant
}  // namespace gui
