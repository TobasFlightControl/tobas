#pragma once

#include <tobas_kdl/tree.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>

#include "./base_setting.hpp"
#include "../robot_info.hpp"

namespace gui
{
namespace setup_assistant
{
class SetupAssistant;

class CustomJointsWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = CustomJointsWidget;
  using super = BaseSettingWidget;

  enum field_t : int
  {
    JOINT_NAME = 0,
    HOME_POSITION = 1,
    MIN_POSITION = 2,
    MAX_POSITION = 3,
    COMMAND_TYPE = 4,
    P_GAIN = 5,
    I_GAIN = 6,
    D_GAIN = 7,
  };

  static constexpr char kJntNameLabel[] = "Joint Name";
  static constexpr char kHomePosLabel[] = "Home Position";
  static constexpr char kMinPosLabel[] = "Min Position";
  static constexpr char kMaxPosLabel[] = "Max Position";
  static constexpr char kCmdTypeLabel[] = "Command Type";
  static constexpr char kPGainLabel[] = "P Gain";
  static constexpr char kIGainLabel[] = "I Gain";
  static constexpr char kDGainLabel[] = "D Gain";

  static constexpr char kPositionLabel[] = "position";
  static constexpr char kVelocityLabel[] = "velocity";
  static constexpr char kEffortLabel[] = "effort";

  static constexpr double kDefaultPGain = 10.;
  static constexpr double kDefaultIGain = 0.1;
  static constexpr double kDefaultDGain = 1.;

  static constexpr int kColWidth = 120;
  static constexpr int kPosDecimals = 3;
  static constexpr int kGainDecimals = 3;

public:
  explicit CustomJointsWidget(const RobotInfo& robot);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onInit() override;
  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  /* ジョイント数． */
  int count() const;

  QString getJointName(int row) const;
  void setJointName(int row, const QString& text);

  double getHomePosition(int row) const;
  void setHomePosition(int row, double value);

  double getMinPosition(int row) const;
  void setMinPosition(int row, double value);

  double getMaxPosition(int row) const;
  void setMaxPosition(int row, double value);

  QString getCommandType(int row) const;
  void setCommandType(int row, const QString& text);

  double getPGain(int row) const;
  void setPGain(int row, double value);

  double getIGain(int row) const;
  void setIGain(int row, double value);

  double getDGain(int row) const;
  void setDGain(int row, double value);

  QStringList getJointNames() const;
  QString getControllerType(int row) const;
  bool pidEnabled(int row) const;

private:
  const RobotInfo& robot_;

  qt::TableWidget* table_;

  int getRow(const QString& jnt_name);
};
};  // namespace setup_assistant
}  // namespace gui
