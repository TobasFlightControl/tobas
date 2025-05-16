#pragma once

#include "../fields/acceptance_radius.hpp"
#include "../fields/altitude.hpp"
#include "../fields/altitude_frame.hpp"
#include "../fields/duration.hpp"
#include "../fields/latitude.hpp"
#include "../fields/longitude.hpp"
#include "./base.hpp"

namespace gui
{
namespace gcs
{
struct WaypointData : public BaseCommandData
{
  using SharedPtr = std::shared_ptr<WaypointData>;

  double latitude;
  double longitude;
  double altitude;
  altitude_frame_t altitude_frame;
  double acceptance_radius;
  double duration;

  command_t type() const
  {
    return command_t::WAYPOINT;
  }
};

class WaypointWidget : public BaseCommandWidget
{
  Q_OBJECT

  using self = WaypointWidget;
  using super = BaseCommandWidget;

public:
  explicit WaypointWidget();

  const char* name() const override;
  BaseCommandData::SharedPtr data() const override;

  double latitude() const;
  double longitude() const;
  double altitude() const;
  altitude_frame_t altitudeFrame() const;
  double acceptanceRadius() const;
  double duration() const;

  void latitude(double value);
  void longitude(double value);
  void altitude(double value);
  void altitudeFrame(altitude_frame_t value);
  void acceptanceRadius(double value);
  void duration(double value);

private:
  field::LatitudeWidget* latitude_;
  field::LongitudeWidget* longitude_;
  field::AltitudeWidget* altitude_;
  field::AltitudeFrameWidget* altitude_frame_;
  field::AcceptanceRadiusWidget* acceptance_radius_;
  field::DurationWidget* duration_;
};
}  // namespace gcs
}  // namespace gui
