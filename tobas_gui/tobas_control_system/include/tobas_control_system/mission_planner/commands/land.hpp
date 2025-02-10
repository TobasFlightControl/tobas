#pragma once

#include "./base.hpp"

namespace gui
{
namespace gcs
{
struct LandData : public BaseCommandData
{
  using SharedPtr = std::shared_ptr<LandData>;

  command_t type() const
  {
    return command_t::LAND;
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
};
}  // namespace gcs
}  // namespace gui
