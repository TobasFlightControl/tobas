#pragma once

#include <yaml-cpp/yaml.h>
#include <QCheckBox>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>

#include "tobas_setup_assistant/robot_info.hpp"
#include "tobas_setup_assistant/signals.hpp"

namespace gui
{
namespace sa
{
namespace hw
{
class DShotWidget : public qt::TableWidget
{
  Q_OBJECT

  using self = DShotWidget;
  using super = qt::TableWidget;

  static constexpr int kTargetNameCol = 0;
  static constexpr int kBidirectionalCol = kTargetNameCol + 1;
  static constexpr int kNumCols = kBidirectionalCol + 1;

  static constexpr char kTargetNameLabel[] = "Target";
  static constexpr char kBidirectionalLabel[] = "Bidirectional";

public:
  explicit DShotWidget(const RobotInfo& robot, const Signals& sig);

  void updateInternalDataStructures();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  void setNumChannels(int num);

  QString targetName(int channel) const;
  bool bidirectional(int channel) const;

  bool contains(const QString& target_name) const;
  int channel(const QString& target_name) const;

private:
  const RobotInfo& robot_;

  tobas::propulsion_system_t prop_type_ = tobas::propulsion_system_t::ELECTRIC;

  QVector<qt::ComboBox*> target_names_;
  QVector<QCheckBox*> bidirectional_;

  void addLastChannel();
  void removeLastChannel();

private Q_SLOTS:
  void onPropulsionTypeChanged(const tobas::propulsion_system_t& new_prop_type);
};
}  // namespace hw
}  // namespace sa
}  // namespace gui
