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

  bool start(const std::string& ns);
  void terminate();

private:
  WindParamsWidget* wind_;
};
}  // namespace sim
}  // namespace gui
