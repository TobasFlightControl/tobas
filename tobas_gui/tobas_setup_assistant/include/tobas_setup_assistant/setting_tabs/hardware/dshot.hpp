#pragma once

#include <yaml-cpp/yaml.h>
#include <QPushButton>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_uadf/model.hpp>

#include "tobas_setup_assistant/signals.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace hw
{
class DShotWidget : public tobas::qt::TableWidget
{
  Q_OBJECT

  using self = DShotWidget;
  using super = tobas::qt::TableWidget;

  static constexpr int kTargetNameCol = 0;
  static constexpr int kBidirectionalCol = kTargetNameCol + 1;
  static constexpr int kNumCols = kBidirectionalCol + 1;

  static constexpr char kTargetNameLabel[] = "Target";
  static constexpr char kBidirectionalLabel[] = "Bidirectional";

public:
  explicit DShotWidget(const uadf::Model& uadf, const Signals& sig);

  void updateInternalDataStructures();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  void setNumChannels(int num);

  QString targetName(int channel) const;
  bool bidirectional(int channel) const;

  bool contains(const QString& target_name) const;
  int channel(const QString& target_name) const;

private:
  const uadf::Model& uadf_;

  tobas::PropulsionSystem prop_type_ = tobas::PropulsionSystem::kElectric;

  tobas::qt::ComboBox* targetNameWidget(int row);
  QPushButton* bidirectionalWidget(int row);

  const tobas::qt::ComboBox* targetNameWidget(int row) const;
  const QPushButton* bidirectionalWidget(int row) const;

  void addLastChannel();
  void removeLastChannel();

  void setBidirectionalButtonChecked(QPushButton* button, bool checked);
  void setBidirectionalButtonText(QPushButton* button, bool checked);

private Q_SLOTS:
  void onPropulsionTypeChanged(const tobas::PropulsionSystem& new_prop_type);
  void onBidirectionalButtonToggled(QPushButton* button, bool checked);
};
}  // namespace hw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
