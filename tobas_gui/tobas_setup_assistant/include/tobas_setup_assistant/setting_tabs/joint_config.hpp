#pragma once

#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_drone_core/joint/joint.hpp>

#include "tobas_setup_assistant/robot_info.hpp"
#include "./base_setting.hpp"
#include "./propulsion_system/propulsion_system.hpp"
#include "./fixed_wing/fixed_wing.hpp"

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

public:
  explicit JointConfigurationWidget(
    const RobotInfo& robot,
    const propulsion_system::PropulsionSystemWidget* propulsion,
    const fixed_wing::FixedWingWidget* fixed_wing);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  // Getters
  QString getLinkName(int row) const;
  QString getJointName(int row) const;
  tobas::jnt_role_t getRole(int row) const;
  tobas::jnt_cmd_iface_t getCommandInterface(int row) const;
  tobas::jnt_hw_iface_t getHardwareInterface(int row) const;
  int getChannel(int row) const;
  double getHomePosition(int row) const;

  // Setters
  void setRole(int row, tobas::jnt_role_t value);
  void setCommandInterface(int row, tobas::jnt_cmd_iface_t value);
  void setHardwareInterface(int row, tobas::jnt_hw_iface_t value);
  void setChannel(int row, int value);
  void setHomePosition(int row, double value);

  /* 登録されているジョイント数． */
  int count() const;

  /* リンク名に対応するテーブルの行を返す．存在しなければ-1を返す． */
  int findLink(const QString& link_name) const;

  /* ジョイント名に対応するテーブルの行を返す．存在しなければ-1を返す． */
  int findJoint(const QString& joint_name) const;

private:
  const RobotInfo& robot_;
  const propulsion_system::PropulsionSystemWidget* propulsion_;
  const fixed_wing::FixedWingWidget* fixed_wing_;

  qt::TableWidget* table_;

  QVector<QLabel*> link_name_;
  QVector<QLabel*> joint_name_;
  QVector<qt::ComboBox*> role_;
  QVector<qt::ComboBox*> cmd_iface_;
  QVector<qt::ComboBox*> hw_iface_;
  QVector<qt::SpinBox*> channel_;
  QVector<qt::DoubleSpinBox*> home_pos_;
  QVector<QLineEdit*> min_pos_;
  QVector<QLineEdit*> max_pos_;
  QVector<QLineEdit*> max_vel_;
  QVector<QLineEdit*> max_eff_;

  QMap<QString, QString> tilt_joint_map_;

  void clear();
  void reset(int row);
  void setDefaultValues(int row);
  void updateEnability(int row);
  void addLink(const std::string& link_name);
  void removeTiltJoint(const QString& rotor_link_name);

private Q_SLOTS:
  void onRoleChanged(int row);
  void onRotorLinkAdded(const QString& link_name);
  void onRotorLinkRemoved(const QString& link_name);
  void onRotorChannelChanged(const QString& link_name, int channel);
  void onRotorIsTiltStateChanged(const QString& link_name, bool is_tilt);
  void onRotorTiltJointNameChanged(const QString& link_name, const QString& tilt_joint_name);
  void onControlSurfaceLinkAdded(const QString& link_name);
  void onControlSurfaceLinkRemoved(const QString& link_name);
};
}  // namespace setup_assistant
}  // namespace gui
