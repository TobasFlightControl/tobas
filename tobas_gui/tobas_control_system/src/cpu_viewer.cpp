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
CPUViewerWidget::CPUViewerWidget(const RosQtBridge& bridge)
{
  temp_ = new qt::HPositionBarWidget();
  temp_->setFixedHeight(kBarHeight);
  temp_->setLower(kMinTemp);
  temp_->setMinimum(kMinTemp);
  temp_->setMaximum(kMaxTemp);

  load_ = new qt::HPositionBarWidget();
  load_->setFixedHeight(kBarHeight);
  load_->setLower(kMinLoad);
  load_->setMinimum(kMinLoad);
  load_->setMaximum(kMaxLoad);

  // Layout
  const auto form = new qt::FormLayout();
  form->addVAlignedRow(new qt::Label("CPU Temperature", kLabelPSize), temp_);
  form->addVAlignedRow(new qt::Label("CPU Load", kLabelPSize), load_);
  setLayout(form);

  // Connection
  connect(&bridge, &RosQtBridge::cpuReceived, this, &self::cpuCb, Qt::QueuedConnection);
}

void CPUViewerWidget::reset()
{
  temp_->setUpper(temp_->getMinimum());
  temp_->setCenterText("");

  load_->setUpper(load_->getMinimum());
  load_->setCenterText("");
}

void CPUViewerWidget::cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu)
{
  temp_->setUpper(cpu->temperature);
  temp_->setCenterText(std::format("{:.0f} ℃", cpu->temperature).c_str());
  if (cpu->temperature > 85.) {
    temp_->setFillColor(Qt::magenta);
  }
  else if (cpu->temperature > 80.) {
    temp_->setFillColor(Qt::red);
  }
  else if (cpu->temperature > 60.) {
    temp_->setFillColor(Qt::yellow);
  }
  else if (cpu->temperature > 0.) {
    temp_->setFillColor(Qt::green);
  }
  else {
    temp_->setFillColor(Qt::blue);
  }

  load_->setUpper(cpu->load * 100);
  load_->setCenterText(std::format("{:.0f} %", cpu->load * 100).c_str());
  if (cpu->load > 80.) {
    load_->setFillColor(Qt::red);
  }
  else if (cpu->load > 60.) {
    load_->setFillColor(Qt::yellow);
  }
  else {
    load_->setFillColor(Qt::green);
  }
}
}  // namespace gcs
}  // namespace gui
