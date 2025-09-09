#pragma once

#include <QLineEdit>

#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace rc
{
class HostnameWidget : public BaseHostWidget
{
  Q_OBJECT

  static constexpr char kHostnameKey[] = "hostname";

public:
  explicit HostnameWidget();

  const char* label() const override;

  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  QString host() const override;

private:
  QLineEdit* hostname_;
};
}  // namespace rc
}  // namespace sa
}  // namespace gui
