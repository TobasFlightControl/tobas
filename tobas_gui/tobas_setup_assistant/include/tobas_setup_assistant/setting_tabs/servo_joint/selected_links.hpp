#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_drone_core/joint/joint_interface.hpp>

#include "tobas_setup_assistant/robot_info.hpp"

namespace gui
{
namespace setup_assistant
{
namespace servo_joint
{
class SelectedLinksWidget : public qt::TableWidget
{
  Q_OBJECT

  using self = SelectedLinksWidget;
  using super = qt::TableWidget;

  static constexpr int kPosDecimals = 3;

  // Columns
  static constexpr int kLinkNameCol = 0;
  static constexpr int kJointNameCol = 1;
  static constexpr int kHomePosCol = 2;
  static constexpr int kMinPosCol = 3;
  static constexpr int kMaxPosCol = 4;
  static constexpr int kInterfaceCol = 5;
  static constexpr int kNumCols = 6;

  // Labels
  static constexpr char kLinkNameLabel[] = "Link Name";
  static constexpr char kJointNameLabel[] = "Joint Name";
  static constexpr char kHomePosLabel[] = "Home Position";
  static constexpr char kMinPosLabel[] = "Min Position";
  static constexpr char kMaxPosLabel[] = "Max Position";
  static constexpr char kInterfaceLabel[] = "Interface";

  static constexpr char kPositionLabel[] = "position";
  static constexpr char kVelocityLabel[] = "velocity";
  static constexpr char kEffortLabel[] = "effort";

public:
  explicit SelectedLinksWidget(const RobotInfo& robot);

  void updateInternalDataStructures();

  YAML::Node dump(const QString& link_name) const;
  void load(const QString& link_name, const YAML::Node& node);

  /* 登録されているジョイント数． */
  int count() const;

  /* 現在選択されているリンク名を返す．存在しない場合は空文字を返す． */
  QString selected() const;

  /* リンク名に対応するテーブルの行を返す．存在しなければ-1を返す． */
  int find(const QString& link_name) const;

  void add(const QString& link_name);
  void remove(const QString& link_name);

  // Getters
  QString linkName(int row) const;
  QString jointName(int row) const;
  double homePosition(int row) const;
  double minPosition(int row) const;
  double maxPosition(int row) const;
  tobas::joint_interface_t interface(int row) const;

  // Setters
  void linkName(int row, const QString& text);
  void jointName(int row, const QString& text);
  void homePosition(int row, double value);
  void minPosition(int row, double value);
  void maxPosition(int row, double value);
  void interface(int row, tobas::joint_interface_t value);

  QStringList linkNames() const;
  QStringList jointNames() const;

private:
  const RobotInfo& robot_;
};
}  // namespace servo_joint
}  // namespace setup_assistant
}  // namespace gui
