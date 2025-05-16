#pragma once

#include <QPushButton>

#include "../param_getters/directory_dialog.hpp"
#include "../param_getters/line_edit.hpp"
#include "../robot_info.hpp"
#include "./base_setting.hpp"

namespace gui
{
namespace sa
{
class RosPackageWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = RosPackageWidget;
  using super = BaseSettingWidget;

  static constexpr int kTextHeight = 50;
  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

Q_SIGNALS:
  void generateButtonClicked();

public:
  explicit RosPackageWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
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
};  // namespace sa
}  // namespace gui
