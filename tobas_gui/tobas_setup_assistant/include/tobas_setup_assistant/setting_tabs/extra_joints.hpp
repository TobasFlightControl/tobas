#pragma once

#include <QLabel>
#include <QPushButton>

#include <tobas_drone_core/joint/command_interface.hpp>
#include <tobas_drone_core/joint/role.hpp>
#include <tobas_kdl/tree.hpp>
#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_uadf/model.hpp>

#include "./base_setting.hpp"

namespace gui
{
namespace sa
{
class ExtraJointsWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = ExtraJointsWidget;
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
  static constexpr char kRoleLabel_UserActive[] = "User Active";
  static constexpr char kRoleLabel_UserPassive[] = "User Passive";

  // Command Interface Labels
  static constexpr char kCmdIfaceLabel_Position[] = "Position";
  static constexpr char kCmdIfaceLabel_Velocity[] = "Velocity";
  static constexpr char kCmdIfaceLabel_Effort[] = "Effort";
  static constexpr char kCmdIfaceLabel_None[] = "None";

public:
  explicit ExtraJointsWidget(const uadf::Model& uadf, const kdl::Tree& tree);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  // Getters
  QString getLinkName(int row) const;
  QString getJointName(int row) const;
  tobas::JointRole getRole(int row) const;
  tobas::JointCommandInterface getCommandInterface(int row) const;
  double getHomePosition(int row) const;  // [rad]

  // Setters
  void setRole(int row, tobas::JointRole value);
  void setCommandInterface(int row, tobas::JointCommandInterface value);
  void setHomePosition(int row, double value);  // [rad]

  /* 登録されているジョイント数． */
  int numJoints() const;

  /* リンク名に対応するテーブルの行を返す．存在しなければ-1を返す． */
  int findLink(const QString& link_name) const;

  /* ジョイント名に対応するテーブルの行を返す．存在しなければ-1を返す． */
  int findJoint(const QString& joint_name) const;

private:
  const uadf::Model& uadf_;
  const kdl::Tree& tree_;

  qt::TableWidget* table_;

  QLabel* linkNameWidget(int row);
  QLabel* jointNameWidget(int row);
  qt::ComboBox* roleWidget(int row);
  qt::ComboBox* commandIfaceWidget(int row);
  qt::SpinBox* homePositionWidget(int row);

  const QLabel* linkNameWidget(int row) const;
  const QLabel* jointNameWidget(int row) const;
  const qt::ComboBox* roleWidget(int row) const;
  const qt::ComboBox* commandIfaceWidget(int row) const;
  const qt::SpinBox* homePositionWidget(int row) const;

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
