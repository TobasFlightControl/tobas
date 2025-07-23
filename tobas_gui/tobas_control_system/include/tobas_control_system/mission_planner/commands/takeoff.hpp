#pragma once

#include "../fields/altitude.hpp"
#include "../fields/altitude_frame.hpp"
#include "../fields/altitude_tolerance.hpp"
#include "../fields/duration.hpp"
#include "./base.hpp"

namespace gui
{
namespace gcs
{
struct TakeoffData : public BaseCommandData
{
  using SharedPtr = std::shared_ptr<TakeoffData>;

  double altitude;
  AltitudeFrame altitude_frame;
  double altitude_tolerance;
  double duration;

  Command type() const
  {
    return Command::kTakeoff;
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

  double altitude() const;
  AltitudeFrame altitudeFrame() const;
  double altitudeTolerance() const;
  double duration() const;

private:
  field::AltitudeWidget* altitude_;
  field::AltitudeFrameWidget* altitude_frame_;
  field::AltitudeToleranceWidget* altitude_tolerance_;
  field::DurationWidget* duration_;
};
}  // namespace gcs
}  // namespace gui
