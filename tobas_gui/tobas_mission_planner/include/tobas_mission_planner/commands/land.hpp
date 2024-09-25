#pragma once

#include "./base.hpp"
#include "../fields/duration.hpp"

namespace gui
{
namespace mission_planner
{
struct LandData : public BaseCommandData
{
  double duration;

  command_t type() const
  {
    return command_t::WAYPOINT;
  }
};

class LandWidget : public BaseCommandWidget
{
  Q_OBJECT

  using self = LandWidget;
  using super = BaseCommandWidget;

public:
  explicit LandWidget();

  const char* name() const override;
  BaseCommandData::SharedPtr data() const override;

private:
  field::DurationWidget* duration_;
};
}  // namespace mission_planner
}  // namespace gui
