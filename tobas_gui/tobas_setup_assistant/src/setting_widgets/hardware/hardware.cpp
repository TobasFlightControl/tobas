// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/hardware/hardware.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/hardware/fc1xx.hpp"
#include "tobas_setup_assistant/setting_tabs/hardware/fc2xx.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace hw
{
namespace
{
constexpr char kTypeKey[] = "hardware_type";
constexpr char kPwmLabel[] = "PWM";
constexpr char kDShotLabel[] = "DShot";
}  // namespace

HardwareWidget::HardwareWidget(const uadf::Model& uadf, const Signals& sig)
{
  type_ = new qt::ComboBox();
  hardwares_ = new qt::StackedWidget();
  pwm_ = new PwmWidget(uadf, sig);
  dshot_ = new DShotWidget(uadf, sig);

  hardwares_->addWidget(new FC1xxWidget());
  hardwares_->addWidget(new FC2xxWidget());

  for (int i = 0; i < hardwares_->count(); ++i) {
    const auto hardware = widget(i);
    type_->addItem(hardware->name());
  }

  // Layout
  const auto pwm_rows = new QVBoxLayout();
  pwm_rows->addWidget(new qt::Label(kPwmLabel, cmn::kLabelPSize, QFont::Bold));
  pwm_rows->addWidget(pwm_);

  const auto dshot_rows = new QVBoxLayout();
  dshot_rows->addWidget(new qt::Label(kDShotLabel, cmn::kLabelPSize, QFont::Bold));
  dshot_rows->addWidget(dshot_);

  const auto rcout_cols = new QHBoxLayout();
  rcout_cols->addLayout(pwm_rows, 1);
  rcout_cols->addLayout(dshot_rows, 1);

  addWidget(type_);
  addSpacing(50);
  addWidget(hardwares_, 0);
  addLayout(rcout_cols, 1);

  // Connection
  connect(type_, qOverload<int>(&qt::ComboBox::currentIndexChanged), this, &self::setCurrentHardware);
}

const char* HardwareWidget::name() const
{
  return "Hardware Interface";
}

const char* HardwareWidget::title() const
{
  return "Configure Hardware Interface";
}

const char* HardwareWidget::description() const
{
  return "Select the FMU you intend to use, "
         "then freely assign each hardware interface to any PWM or DShot channel.";
}

void HardwareWidget::updateInternalDataStructures()
{
  pwm_->updateInternalDataStructures();
  dshot_->updateInternalDataStructures();
}

void HardwareWidget::setToDefaults()
{
  for (int i = 0; i < hardwares_->count(); ++i) {
    const auto hardware = widget(i);
    hardware->setToDefaults();
  }

  setCurrentHardware(0);
}

bool HardwareWidget::isValid()
{
  if (!selected()->isValid()) {
    return false;
  }

  if (!pwm_->isValid()) {
    return false;
  }
  if (!dshot_->isValid()) {
    return false;
  }

  return true;
}

YAML::Node HardwareWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kTypeKey] = type_->currentText();

  for (int i = 0; i < hardwares_->count(); ++i) {
    const auto hardware = widget(i);
    node[hardware->name()] = hardware->dump();
  }

  node[kPwmLabel] = pwm_->dump();
  node[kDShotLabel] = dshot_->dump();

  return node;
}

void HardwareWidget::load(const YAML::Node& node)
{
  type_->setCurrentText(node[kTypeKey].as<QString>());
  setCurrentHardware(type_->currentIndex());

  for (int i = 0; i < hardwares_->count(); ++i) {
    const auto hardware = widget(i);
    hardware->load(node[hardware->name()]);
  }

  pwm_->load(node[kPwmLabel]);
  dshot_->load(node[kDShotLabel]);
}

const PwmWidget* HardwareWidget::pwm() const
{
  return pwm_;
}

const DShotWidget* HardwareWidget::dshot() const
{
  return dshot_;
}

const char* HardwareWidget::fmuName() const
{
  return selected()->name();
}

const char* HardwareWidget::hardwarePackage() const
{
  return selected()->hardwarePackage();
}

double HardwareWidget::gyroNoiseDensity() const
{
  return selected()->gyroNoiseDensity();
}

double HardwareWidget::gyroRandomWalk() const
{
  return selected()->gyroRandomWalk();
}

int HardwareWidget::gyroBiasCorrTime() const
{
  return selected()->gyroBiasCorrTime();
}

double HardwareWidget::accNoiseDensity() const
{
  return selected()->accNoiseDensity();
}

double HardwareWidget::accRandomWalk() const
{
  return selected()->accRandomWalk();
}

int HardwareWidget::accBiasCorrTime() const
{
  return selected()->accBiasCorrTime();
}

int HardwareWidget::magUpdateRate() const
{
  return selected()->magUpdateRate();
}

double HardwareWidget::magNoiseStddev() const
{
  return selected()->magNoiseStddev();
}

double HardwareWidget::magHardBiasNorm() const
{
  return selected()->magHardBiasNorm();
}

int HardwareWidget::presUpdateRate() const
{
  return selected()->presUpdateRate();
}

double HardwareWidget::presNoiseStddev() const
{
  return selected()->presNoiseStddev();
}

int HardwareWidget::gnssUpdateRate() const
{
  return selected()->gnssUpdateRate();
}

double HardwareWidget::gnssHorizontalPositionAccuracy() const
{
  return selected()->gnssHorizontalPositionAccuracy();
}

double HardwareWidget::gnssVerticalPositionAccuracy() const
{
  return selected()->gnssVerticalPositionAccuracy();
}

double HardwareWidget::gnssHorizontalVelocityStddev() const
{
  return selected()->gnssHorizontalVelocityStddev();
}

double HardwareWidget::gnssVerticalVelocityStddev() const
{
  return selected()->gnssVerticalVelocityStddev();
}

int HardwareWidget::numPwmChannels() const
{
  return selected()->numPwmChannels();
}

int HardwareWidget::numDShotChannels() const
{
  return selected()->numDShotChannels();
}

bool HardwareWidget::hasRpmFilter() const
{
  return selected()->hasRpmFilter();
}

BaseHardwareWidget* HardwareWidget::widget(int index)
{
  return qt::qPointerCast<BaseHardwareWidget>(hardwares_->widget(index));
}

const BaseHardwareWidget* HardwareWidget::widget(int index) const
{
  return qt::qConstPointerCast<BaseHardwareWidget>(hardwares_->widget(index));
}

BaseHardwareWidget* HardwareWidget::selected()
{
  return widget(hardwares_->currentIndex());
}

const BaseHardwareWidget* HardwareWidget::selected() const
{
  return widget(hardwares_->currentIndex());
}

void HardwareWidget::setCurrentHardware(int index)
{
  hardwares_->setCurrentIndex(index);

  const auto hardware = widget(index);
  pwm_->setNumChannels(hardware->numPwmChannels());
  dshot_->setNumChannels(hardware->numDShotChannels());
}

void HardwareWidget::onHardwareTypeChanged(int index)
{
  qDebug().nospace() << "HardwareWidget::onHardwareTypeChanged(" << index << ")";

  setCurrentHardware(index);
}
}  // namespace hw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
