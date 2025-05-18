#include "tobas_control_system/cpu_viewer.hpp"

#include <format>

#include <tobas_constants/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace gui
{
namespace gcs
{
CPUViewerWidget::CPUViewerWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  temp_ = new qt::HPositionBarWidget();
  load_ = new qt::HPositionBarWidget();

  temp_->setFixedHeight(kBarHeight);
  load_->setFixedHeight(kBarHeight);

  // Layout
  const auto form = new qt::FormLayout();
  form->addVAlignedRow(new qt::Label("CPU Temperature", kLabelPSize), temp_);
  form->addVAlignedRow(new qt::Label("CPU Load", kLabelPSize), load_);
  setLayout(form);

  // Connection
  connect(this, &self::cpuReceived, this, &self::cpuCbQt);
}

void CPUViewerWidget::reset()
{
  temp_->setUpper(temp_->getMinimum());
  temp_->setCenterText("");

  load_->setUpper(load_->getMinimum());
  load_->setCenterText("");
}

void CPUViewerWidget::updateNamespace(const std::string& ns)
{
  reset();

  temp_->setLower(kMinTemp);
  temp_->setMinimum(kMinTemp);
  temp_->setMaximum(kMaxTemp);

  load_->setLower(kMinLoad);
  load_->setMinimum(kMinLoad);
  load_->setMaximum(kMaxLoad);

  cpu_sub_ =
    ros2::createSubscriber(node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kCpuTopic), &self::cpuCbRos, this);
}

void CPUViewerWidget::cpuCbRos(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu)
{
  Q_EMIT cpuReceived(cpu->temperature, cpu->load);
}

void CPUViewerWidget::cpuCbQt(double temp, double load)
{
  temp_->setUpper(temp);
  temp_->setCenterText(std::format("{:.0f} ℃", temp).c_str());
  if (temp > 85.) {
    temp_->setFillColor(Qt::magenta);
  }
  else if (temp > 80.) {
    temp_->setFillColor(Qt::red);
  }
  else if (temp > 60.) {
    temp_->setFillColor(Qt::yellow);
  }
  else if (temp > 0.) {
    temp_->setFillColor(Qt::green);
  }
  else {
    temp_->setFillColor(Qt::blue);
  }

  load_->setUpper(load * 100);
  load_->setCenterText(std::format("{:.0f} %", load * 100).c_str());
  if (load > 80.) {
    load_->setFillColor(Qt::red);
  }
  else if (load > 60.) {
    load_->setFillColor(Qt::yellow);
  }
  else {
    load_->setFillColor(Qt::green);
  }
}

}  // namespace gcs
}  // namespace gui
