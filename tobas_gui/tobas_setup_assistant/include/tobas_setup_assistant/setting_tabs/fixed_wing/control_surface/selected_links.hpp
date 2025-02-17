#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/widgets/table_widget.hpp>

#include "tobas_setup_assistant/robot_info.hpp"

namespace gui
{
namespace sa
{
namespace fixed_wing
{
class SelectedLinksWidget : public qt::TableWidget
{
  Q_OBJECT

  using self = SelectedLinksWidget;
  using super = qt::TableWidget;

  static constexpr int kColWidth = 120;
  static constexpr double kAngleLimit = M_PI_4;

  // Columns
  static constexpr int kLinkNameCol = 0;
  static constexpr int kJointNameCol = kLinkNameCol + 1;
  static constexpr int kLiftCoefCol = kJointNameCol + 1;
  static constexpr int kDragCoefCol = kLiftCoefCol + 1;
  static constexpr int kSideCoefCol = kDragCoefCol + 1;
  static constexpr int kRollCoefCol = kSideCoefCol + 1;
  static constexpr int kPitchCoefCol = kRollCoefCol + 1;
  static constexpr int kYawCoefCol = kPitchCoefCol + 1;
  static constexpr int kNumCols = kYawCoefCol + 1;

  // Labels
  static constexpr char kLinkNameLabel[] = "Link Name";
  static constexpr char kJointNameLabel[] = "Joint Name";
  static constexpr char kLiftCoefLabel[] = "Lift Coef";
  static constexpr char kDragCoefLabel[] = "Drag Coef";
  static constexpr char kSideCoefLabel[] = "Side Coef";
  static constexpr char kRollCoefLabel[] = "Roll Coef";
  static constexpr char kPitchCoefLabel[] = "Pitch Coef";
  static constexpr char kYawCoefLabel[] = "Yaw Coef";

public:
  explicit SelectedLinksWidget(const RobotInfo& robot);

  void updateInternalDataStructures();

  YAML::Node dump(const QString& link_name) const;
  void load(const QString& link_name, const YAML::Node& node);

  /* 登録されている制御面の個数． */
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
  double liftCoef(int row) const;
  double dragCoef(int row) const;
  double sideCoef(int row) const;
  double rollCoef(int row) const;
  double pitchCoef(int row) const;
  double yawCoef(int row) const;

  // Setters
  void linkName(int row, const QString& text);
  void jointName(int row, const QString& text);
  void liftCoef(int row, double value);
  void dragCoef(int row, double value);
  void sideCoef(int row, double value);
  void rollCoef(int row, double value);
  void pitchCoef(int row, double value);
  void yawCoef(int row, double value);

  QStringList linkNames() const;
  QStringList jointNames() const;

private:
  const RobotInfo& robot_;
};
}  // namespace fixed_wing
}  // namespace sa
}  // namespace gui
