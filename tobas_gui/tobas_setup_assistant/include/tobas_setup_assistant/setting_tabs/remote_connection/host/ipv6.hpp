#pragma once

#include <tobas_qt_tools/widgets/ipv6_edit.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace rc
{
class IPv6Widget : public BaseHostWidget
{
  Q_OBJECT

  static constexpr char kAddressKey[] = "address";

public:
  explicit IPv6Widget();

  const char* label() const override;

  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  QString host() const override;

private:
  qt::IPv6Edit* ipv6_;
};
}  // namespace rc
}  // namespace sa
}  // namespace gui
}  // namespace tobas
