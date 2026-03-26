#pragma once

#include <QCheckBox>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace ctrl
{
class PlanarMulticopterWidget : public BaseControllerWidget
{
  Q_OBJECT

public:
  explicit PlanarMulticopterWidget();

  FrameType frameType() const override;
  QString controllerPackage() const override;
  QString pluginName() const override;

  tobas::RcCommand acrobatModeCommand() const override;
  tobas::RcCommand stabilizeModeCommand() const override;
  tobas::RcCommand loiterModeCommand() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isValid() override;

private:
  QCheckBox* do_dist_comp_trans_;
  QCheckBox* do_dist_comp_rot_;
};
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
}  // namespace tobas
