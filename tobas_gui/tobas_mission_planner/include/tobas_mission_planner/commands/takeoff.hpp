#pragma once

#include "./base.hpp"
#include "../fields/altitude.hpp"
#include "../fields/altitude_frame.hpp"
#include "../fields/acceptance_radius.hpp"
#include "../fields/duration.hpp"

namespace gui
{
namespace mission_planner
{
struct TakeoffData : public BaseCommandData
{
  double altitude;
  altitude_frame_t altitude_frame;
  double acceptance_radius;
  double duration;

  command_t type() const
  {
    return command_t::TAKEOFF;
  }
};

class TakeoffWidget : public BaseCommandWidget
{
  Q_OBJECT

  using self = TakeoffWidget;
  using super = BaseCommandWidget;

public:
  explicit TakeoffWidget();

  const char* name() const override;
  BaseCommandData::SharedPtr data() const override;

private:
  field::AltitudeWidget* altitude_;
  field::AltitudeFrameWidget* altitude_frame_;
  field::AcceptanceRadiusWidget* acceptance_radius_;
  field::DurationWidget* duration_;
};
}  // namespace mission_planner
}  // namespace gui
