#pragma once

#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_drone_core/joint/joint.hpp>

#include "tobas_setup_assistant/robot_info.hpp"
#include "./base_setting.hpp"

namespace gui
{
namespace setup_assistant
{
class JointConfigurationWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = JointConfigurationWidget;
  using super = BaseSettingWidget;

  static constexpr int kPosDecimals = 3;
  static constexpr int kVelDecimals = 3;
  static constexpr int kEffDecimals = 3;

  // Columns
  static constexpr int kLinkNameCol = 0;
  static constexpr int kJointNameCol = kLinkNameCol + 1;
  static constexpr int kRoleCol = kJointNameCol + 1;
  static constexpr int kCmdIfaceCol = kRoleCol + 1;
  static constexpr int kHwIfaceCol = kCmdIfaceCol + 1;
  static constexpr int kChannelCol = kHwIfaceCol + 1;
  static constexpr int kHomePosCol = kChannelCol + 1;
  static constexpr int kMinPosCol = kHomePosCol + 1;
  static constexpr int kMaxPosCol = kMinPosCol + 1;
  static constexpr int kMaxVelCol = kMaxPosCol + 1;
  static constexpr int kMaxEffCol = kMaxVelCol + 1;
  static constexpr int kNumCols = kMaxEffCol + 1;

  // Field Labels
  static constexpr char kLinkNameLabel[] = "Link Name";
  static constexpr char kJointNameLabel[] = "Joint Name";
  static constexpr char kRoleLabel[] = "Role";
  static constexpr char kCmdIfaceLabel[] = "Command Interface";
  static constexpr char kHwIfaceLabel[] = "Hardware Interface";
  static constexpr char kChannelLabel[] = "Channel";
  static constexpr char kHomePosLabel[] = "Home Position";
  static constexpr char kMinPosLabel[] = "Min Position";
  static constexpr char kMaxPosLabel[] = "Max Position";
  static constexpr char kMaxVelLabel[] = "Max Velocity";
  static constexpr char kMaxEffLabel[] = "Max Effort";

  // Role Labels
  static constexpr char kRoleLabel_rotor[] = "Rotor";
  static constexpr char kRoleLabel_tilt[] = "Tilt Joint";
  static constexpr char kRoleLabel_cs[] = "Control Surface";
  static constexpr char kRoleLabel_manip[] = "Manipulation";
  static constexpr char kRoleLabel_wheel[] = "Wheel";
  static constexpr char kRoleLabel_other[] = "Other";

  // Command Interface Labels
  static constexpr char kCmdIfaceLabel_pos[] = "position";
  static constexpr char kCmdIfaceLabel_vel[] = "velocity";
  static constexpr char kCmdIfaceLabel_eff[] = "effort";

  // Hardware Interface Labels
  static constexpr char kHwIfaceLabel_pwm[] = "PWM";
  static constexpr char kHwIfaceLabel_other[] = "Other";

public:
  explicit JointConfigurationWidget(const RobotInfo& robot);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  // Getters
  QString linkName(int row) const;
  QString jointName(int row) const;
  tobas::jnt_role_t role(int row) const;
  tobas::jnt_cmd_iface_t commandInterface(int row) const;
  tobas::jnt_hw_iface_t hardwareInterface(int row) const;
  int channel(int row) const;
  double homePosition(int row) const;

  // Setters
  void role(int row, tobas::jnt_role_t value) const;
  void commandInterface(int row, tobas::jnt_cmd_iface_t value) const;
  void hardwareInterface(int row, tobas::jnt_hw_iface_t value) const;
  void channel(int row, int value) const;
  void homePosition(int row, double value) const;

  /* 登録されているジョイント数． */
  int count() const;

  /* リンク名に対応するテーブルの行を返す．存在しなければ-1を返す． */
  int findLink(const QString& link_name) const;

  /* ジョイント名に対応するテーブルの行を返す．存在しなければ-1を返す． */
  int findJoint(const QString& joint_name) const;

private:
  const RobotInfo& robot_;

  qt::TableWidget* table_;

  void addLink(const std::string& link_name);
};
}  // namespace setup_assistant
}  // namespace gui
