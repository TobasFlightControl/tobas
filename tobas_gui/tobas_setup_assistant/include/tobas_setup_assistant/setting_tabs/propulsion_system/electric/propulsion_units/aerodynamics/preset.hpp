#pragma once

#include <filesystem>

#include <eigen3/Eigen/Core>

#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "../propeller.hpp"
#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
class AerodynamicsWidget_Preset : public AerodynamicsWidget_Base
{
  Q_OBJECT

  static constexpr char kPropellerNameKey[] = "propeller_name";

  enum DataType
  {
    kThrustStand,
    kUiuc,
  };

public:
  explicit AerodynamicsWidget_Preset(const PropellerWidget* propeller);

  const char* name() const override;
  const char* description() const override;

  bool isValid() override;
  void copyFrom(const AerodynamicsWidget_Base* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  double motorConst() const override;
  double momentConst() const override;
  double dragConst() const override;

private:
  const PropellerWidget* const propeller_;

  qt::ComboBox* data_name_;

  std::filesystem::path getCurrentDataPath() const;
  DataType getCurrentDataType() const;

  std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
  parseThrustStandData(const std::filesystem::path& path) const;
  std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> parseUiucData(const std::filesystem::path& path) const;

  static std::filesystem::path dataDir();
  static std::filesystem::path thrustStandDataDir();
  static std::filesystem::path uiucDataDir();
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
