#pragma once

#include "./base.hpp"

namespace gui
{
namespace ctrl
{
struct LandData : public BaseCommandData
{
  using SharedPtr = std::shared_ptr<LandData>;

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

private:
};
}  // namespace ctrl
}  // namespace gui
