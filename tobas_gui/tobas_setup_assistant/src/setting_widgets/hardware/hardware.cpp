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

namespace tobas
{
namespace gui
{
namespace sa
{
namespace hw
{
HardwareWidget::HardwareWidget(const uadf::Model& uadf, const Signals& sig)
{
  type_ = new qt::ComboBox();
  hardwares_ = new qt::StackedWidget();
  pwm_ = new PwmWidget(uadf, sig);
  dshot_ = new DShotWidget(uadf, sig);

  hardwares_->addWidget(new T1Widget());

  for (int i = 0; i < hardwares_->count(); ++i) {
    const auto hardware = qt::qConstPointerCast<BaseHardwareWidget>(hardwares_->widget(i));
    type_->addItem(hardware->name());
  }

  setCurrentHardware(0);

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
  connect(type_, QOverload<int>::of(&qt::ComboBox::currentIndexChanged), this, &self::setCurrentHardware);
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
    const auto hardware = qt::qConstPointerCast<BaseHardwareWidget>(hardwares_->widget(i));
    node[hardware->name()] = hardware->dump();
  }

  node[kPwmLabel] = pwm_->dump();
  node[kDShotLabel] = dshot_->dump();

  return node;
}

void HardwareWidget::load(const YAML::Node& node)
{
  type_->setCurrentText(node[kTypeKey].as<QString>());

  for (int i = 0; i < hardwares_->count(); ++i) {
    const auto hardware = qt::qPointerCast<BaseHardwareWidget>(hardwares_->widget(i));
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

int HardwareWidget::imuUpdateRate() const
{
  return selected()->imuUpdateRate();
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

void HardwareWidget::setCurrentHardware(int index)
{
  hardwares_->setCurrentIndex(index);

  const auto hardware = qt::qConstPointerCast<BaseHardwareWidget>(hardwares_->widget(index));
  pwm_->setNumChannels(hardware->numPwmChannels());
  dshot_->setNumChannels(hardware->numDShotChannels());
}

BaseHardwareWidget* HardwareWidget::selected()
{
  return qt::qPointerCast<BaseHardwareWidget>(hardwares_->currentWidget());
}

const BaseHardwareWidget* HardwareWidget::selected() const
{
  return qt::qConstPointerCast<BaseHardwareWidget>(hardwares_->currentWidget());
}
}  // namespace hw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
