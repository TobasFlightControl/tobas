#pragma once

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
class SpeedLimitWidget_Manual : public SpeedLimitWidget_Base
{
  Q_OBJECT

public:
  const char* name() const override;

  void onInit() override;

  bool isValid() override;

  double maxRotSpeed() const override;
};
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
