#pragma once

#include <QLabel>
#include <QPushButton>

#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_drone_core/joint/joint.hpp>

#include "tobas_setup_assistant/robot_info.hpp"
#include "./base_setting.hpp"
#include "./propulsion_system/propulsion_system.hpp"
#include "./fixed_wing/fixed_wing.hpp"

namespace gui
{
namespace sa
{
class JointConfigurationWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = JointConfigurationWidget;
  using super = BaseSettingWidget;

  // Columns
  static constexpr int kLinkNameCol = 0;
  static constexpr int kJointNameCol = kLinkNameCol + 1;
  static constexpr int kRoleCol = kJointNameCol + 1;
  static constexpr int kCmdIfaceCol = kRoleCol + 1;
  static constexpr int kHwIfaceCol = kCmdIfaceCol + 1;
  static constexpr int kHomePosCol = kHwIfaceCol + 1;
  static constexpr int kPwmChannelCol = kHomePosCol + 1;
  static constexpr int kPwmMinPeriodCol = kPwmChannelCol + 1;
  static constexpr int kPwmMaxPeriodCol = kPwmMinPeriodCol + 1;
  static constexpr int kPwmMinAngleCol = kPwmMaxPeriodCol + 1;
  static constexpr int kPwmMaxAngleCol = kPwmMinAngleCol + 1;
  static constexpr int kPwmReverseCol = kPwmMaxAngleCol + 1;
  static constexpr int kNumCols = kPwmReverseCol + 1;

  // Field Labels
  static constexpr char kLinkNameLabel[] = "Link Name";
  static constexpr char kJointNameLabel[] = "Joint Name";
  static constexpr char kRoleLabel[] = "Role";
  static constexpr char kCmdIfaceLabel[] = "Command Interface";
  static constexpr char kHwIfaceLabel[] = "Hardware Interface";
  static constexpr char kHomePosLabel[] = "Home Position";
  static constexpr char kPwmChannelLabel[] = "PWM Channel";
  static constexpr char kPwmMinPeriodLabel[] = "PWM Min Period";
  static constexpr char kPwmMaxPeriodLabel[] = "PWM Max Period";
  static constexpr char kPwmMinAngleLabel[] = "PWM Min Angle";
  static constexpr char kPwmMaxAngleLabel[] = "PWM Max Angle";
  static constexpr char kPwmReverseLabel[] = "PWM Reverse";

  // Role Labels
  static constexpr char kRoleLabel_Rotor[] = "Rotor";
  static constexpr char kRoleLabel_TiltJoint[] = "Tilt Joint";
  static constexpr char kRoleLabel_ControlSurface[] = "Control Surface";
  static constexpr char kRoleLabel_Manipulation[] = "Manipulation";
  static constexpr char kRoleLabel_PassiveWheel[] = "Passive Wheel";
  static constexpr char kRoleLabel_Other[] = "Other";

  // Command Interface Labels
  static constexpr char kCmdIfaceLabel_Position[] = "Position";
  static constexpr char kCmdIfaceLabel_Velocity[] = "Velocity";
  static constexpr char kCmdIfaceLabel_Effort[] = "Effort";
  static constexpr char kCmdIfaceLabel_None[] = "None";

  // Hardware Interface Labels
  static constexpr char kHwIfaceLabel_PWM[] = "PWM";
  static constexpr char kHwIfaceLabel_Other[] = "Other";

  // Reverse Labels
  static constexpr char kReverseLabel_Normal[] = "Normal";
  static constexpr char kReverseLabel_Reverse[] = "Reverse";

public:
  explicit JointConfigurationWidget(
    const RobotInfo& robot,
    const propulsion::PropulsionSystemWidget* propulsion,
    const fixed_wing::FixedWingWidget* fixed_wing);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  // Getters
  QString getLinkName(int row) const;
  QString getJointName(int row) const;
  tobas::jnt_role_t getRole(int row) const;
  tobas::jnt_cmd_iface_t getCommandInterface(int row) const;
  tobas::hw_iface_t getHardwareInterface(int row) const;
  double getHomePosition(int row) const;  // [rad]
  int getPwmChannel(int row) const;
  uint16_t getPwmMinPeriod(int row) const;  // [us]
  uint16_t getPwmMaxPeriod(int row) const;  // [us]
  double getPwmMinAngle(int row) const;     // [rad]
  double getPwmMaxAngle(int row) const;     // [rad]
  bool getPwmReverse(int row) const;

  // Setters
  void setRole(int row, tobas::jnt_role_t value);
  void setCommandInterface(int row, tobas::jnt_cmd_iface_t value);
  void setHardwareInterface(int row, tobas::hw_iface_t value);
  void setHomePosition(int row, double value);  // [rad]
  void setPwmChannel(int row, int value);
  void setPwmMinPeriod(int row, uint16_t value);  // [us]
  void setPwmMaxPeriod(int row, uint16_t value);  // [us]
  void setPwmMinAngle(int row, double value);     // [rad]
  void setPwmMaxAngle(int row, double value);     // [rad]
  void setPwmReverse(int row, bool value);

  /* 登録されているジョイント数． */
  int numJoints() const;

  /* リンク名に対応するテーブルの行を返す．存在しなければ-1を返す． */
  int findLink(const QString& link_name) const;

  /* ジョイント名に対応するテーブルの行を返す．存在しなければ-1を返す． */
  int findJoint(const QString& joint_name) const;

private:
  const RobotInfo& robot_;
  const propulsion::PropulsionSystemWidget* propulsion_;
  const fixed_wing::FixedWingWidget* fixed_wing_;

  qt::TableWidget* table_;

  QVector<QLabel*> link_name_;
  QVector<QLabel*> joint_name_;
  QVector<qt::ComboBox*> role_;
  QVector<qt::ComboBox*> cmd_iface_;
  QVector<qt::ComboBox*> hw_iface_;
  QVector<qt::SpinBox*> home_pos_;  // [deg]
  QVector<qt::SpinBox*> pwm_channel_;
  QVector<qt::SpinBox*> pwm_min_period_;  // [us]
  QVector<qt::SpinBox*> pwm_max_period_;  // [us]
  QVector<qt::SpinBox*> pwm_min_angle_;   // [deg]
  QVector<qt::SpinBox*> pwm_max_angle_;   // [deg]
  QVector<QPushButton*> pwm_reverse_;

  QMap<QString, QString> tilt_joint_map_;

  void clear();
  void reset(int row);
  void setDefaultValues(int row);
  void updateEnability(int row);
  void addLink(const std::string& link_name);
  void removeTiltJoint(const QString& rotor_link_name);

private Q_SLOTS:
  void onRoleChanged(int row);
  void onHardwareInterfaceChanged(int row);
  void onRotorLinkAdded(const QString& link_name);
  void onRotorLinkRemoved(const QString& link_name);
  void onRotorIsTiltStateChanged(const QString& link_name, bool is_tilt);
  void onRotorTiltJointNameChanged(const QString& link_name, const QString& tilt_joint_name);
  void onControlSurfaceLinkAdded(const QString& link_name);
  void onControlSurfaceLinkRemoved(const QString& link_name);
};
}  // namespace sa
}  // namespace gui
