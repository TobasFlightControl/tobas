#pragma once

#include "../fields/acceptance_radius.hpp"
#include "../fields/altitude.hpp"
#include "../fields/altitude_frame.hpp"
#include "../fields/duration.hpp"
#include "./base.hpp"

namespace gui
{
namespace ctrl
{
struct ReturnToHomeData : public BaseCommandData
{
  using SharedPtr = std::shared_ptr<ReturnToHomeData>;

  double altitude;
  AltitudeFrame altitude_frame;
  double acceptance_radius;
  double duration;

  Command type() const
  {
    return Command::kReturnToHome;
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
  AltitudeFrame altitudeFrame() const;
  double acceptanceRadius() const;
  double duration() const;

private:
  field::AltitudeWidget* altitude_;
  field::AltitudeFrameWidget* altitude_frame_;
  field::AcceptanceRadiusWidget* acceptance_radius_;
  field::DurationWidget* duration_;
};
}  // namespace ctrl
}  // namespace gui
