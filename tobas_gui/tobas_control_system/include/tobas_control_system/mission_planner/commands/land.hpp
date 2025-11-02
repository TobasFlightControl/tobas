#pragma once

#include "../fields/land_speed.hpp"
#include "./base.hpp"

namespace gui
{
namespace ctrl
{
struct LandData : public BaseCommandData
{
  using SharedPtr = std::shared_ptr<LandData>;

  double speed;  // [m/s]

  Command type() const
  {
    return Command::kLand;
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

  double speed() const;

  void speed(double value);

private:
  field::LandSpeedWidget* speed_;
};
}  // namespace ctrl
}  // namespace gui
