#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/widgets/table_widget.hpp>

#include "tobas_setup_assistant/robot_info.hpp"

namespace gui
{
namespace setup_assistant
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
  static constexpr int kJointNameCol = 1;
  static constexpr int kMinAngleCol = 2;
  static constexpr int kMaxAngleCol = 3;
  static constexpr int kMaxAngleRateCol = 4;
  static constexpr int kLiftCoefCol = 5;
  static constexpr int kDragCoefCol = 6;
  static constexpr int kSideCoefCol = 7;
  static constexpr int kRollCoefCol = 8;
  static constexpr int kPitchCoefCol = 9;
  static constexpr int kYawCoefCol = 10;
  static constexpr int kNumCols = 11;

  // Labels
  static constexpr char kLinkNameLabel[] = "Link Name";
  static constexpr char kJointNameLabel[] = "Joint Name";
  static constexpr char kMinAngleLabel[] = "Min Angle";
  static constexpr char kMaxAngleLabel[] = "Max Angle";
  static constexpr char kMaxAngleRateLabel[] = "Max Angle Rate";
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

  /* 現在選択されているリンク名を返す．存在しない場合は空文字を返す． */
  QString selected() const;

  /* リンク名に対応するテーブルの行を返す．存在しなければ-1を返す． */
  int find(const QString& link_name) const;

  void add(const QString& link_name);
  void remove(const QString& link_name);

  // Getters
  QString linkName(int row) const;
  QString jointName(int row) const;
  double minAngle(int row) const;
  double maxAngle(int row) const;
  double maxAngleRate(int row) const;
  double liftCoef(int row) const;
  double dragCoef(int row) const;
  double sideCoef(int row) const;
  double rollCoef(int row) const;
  double pitchCoef(int row) const;
  double yawCoef(int row) const;

  // Setters
  void linkName(int row, const QString& text);
  void jointName(int row, const QString& text);
  void minAngle(int row, double value);
  void maxAngle(int row, double value);
  void maxAngleRate(int row, double value);
  void liftCoef(int row, double value);
  void dragCoef(int row, double value);
  void sideCoef(int row, double value);
  void rollCoef(int row, double value);
  void pitchCoef(int row, double value);
  void yawCoef(int row, double value);

private:
  const RobotInfo& robot_;
};
}  // namespace fixed_wing
}  // namespace setup_assistant
}  // namespace gui
