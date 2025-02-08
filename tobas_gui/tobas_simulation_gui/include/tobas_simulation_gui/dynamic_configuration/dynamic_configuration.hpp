#pragma once

#include "./wind_parameters.hpp"

namespace gui
{
namespace sim
{
class DynamicConfigWidget : public QWidget
{
  Q_OBJECT

  using self = DynamicConfigWidget;
  using super = QWidget;

public:
  explicit DynamicConfigWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

  bool start();
  void reset();

private:
  WindParamsWidget* wind_params_;
};
}  // namespace sim
}  // namespace gui
