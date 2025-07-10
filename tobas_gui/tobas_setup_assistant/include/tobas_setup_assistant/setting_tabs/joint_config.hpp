#pragma once

#include <QLabel>
#include <QPushButton>

#include <tobas_drone_core/joint/joint.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>

#include "./base_setting.hpp"
#include "./fixed_wing/fixed_wing.hpp"
#include "./propulsion_system/propulsion_system.hpp"
#include "tobas_setup_assistant/robot_info.hpp"

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
  static constexpr int kHomePosCol = kCmdIfaceCol + 1;
  static constexpr int kNumCols = kHomePosCol + 1;

  // Field Labels
  static constexpr char kLinkNameLabel[] = "Link Name";
  static constexpr char kJointNameLabel[] = "Joint Name";
  static constexpr char kRoleLabel[] = "Role";
  static constexpr char kCmdIfaceLabel[] = "Command Interface";
  static constexpr char kHomePosLabel[] = "Home Position";

  // Role Labels
  static constexpr char kRoleLabel_LandingGear[] = "Landing Gear";
  static constexpr char kRoleLabel_PassiveWheel[] = "Passive Wheel";
  static constexpr char kRoleLabel_Manipulation[] = "Manipulation";
  static constexpr char kRoleLabel_Other[] = "Other";

  // Command Interface Labels
  static constexpr char kCmdIfaceLabel_Position[] = "Position";
  static constexpr char kCmdIfaceLabel_Velocity[] = "Velocity";
  static constexpr char kCmdIfaceLabel_Effort[] = "Effort";
  static constexpr char kCmdIfaceLabel_None[] = "None";

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
  double getHomePosition(int row) const;  // [rad]

  // Setters
  void setRole(int row, tobas::jnt_role_t value);
  void setCommandInterface(int row, tobas::jnt_cmd_iface_t value);
  void setHomePosition(int row, double value);  // [rad]

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
  QVector<qt::SpinBox*> home_pos_;  // [deg]

  void clear();
  void reset(int row);
  void setDefaultValues(int row);
  void updateEnability(int row);
  void addLink(const std::string& link_name);

private Q_SLOTS:
  void onRoleChanged(int row);
};
}  // namespace sa
}  // namespace gui
