#pragma once

#include "../fields/acceptance_radius.hpp"
#include "../fields/altitude.hpp"
#include "../fields/altitude_frame.hpp"
#include "../fields/duration.hpp"
#include "./base.hpp"

namespace gui
{
namespace gcs
{
struct ReturnToHomeData : public BaseCommandData
{
  using SharedPtr = std::shared_ptr<ReturnToHomeData>;

  double altitude;
  altitude_frame_t altitude_frame;
  double acceptance_radius;
  double duration;

  command_t type() const
  {
    return command_t::RETURN_TO_HOME;
  }
};

class ReturnToHomeWidget : public BaseCommandWidget
{
  Q_OBJECT

  using self = ReturnToHomeWidget;
  using super = BaseCommandWidget;

public:
  explicit ReturnToHomeWidget();

  const char* name() const override;
  BaseCommandData::SharedPtr data() const override;

  double altitude() const;
  altitude_frame_t altitudeFrame() const;
  double acceptanceRadius() const;
  double duration() const;

private:
  field::AltitudeWidget* altitude_;
  field::AltitudeFrameWidget* altitude_frame_;
  field::AcceptanceRadiusWidget* acceptance_radius_;
  field::DurationWidget* duration_;
};
}  // namespace gcs
}  // namespace gui
