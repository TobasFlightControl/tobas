// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/hardware/hardware.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/hardware/fc1xx.hpp"
#include "tobas_setup_assistant/setting_tabs/hardware/fc2xx.hpp"

namespace tobas::gui::sa::hw
{
namespace
{
constexpr char kTypeKey[] = "hardware_type";
constexpr char kPwmLabel[] = "PWM";
constexpr char kDShotLabel[] = "DShot";
}  // namespace

HardwareWidget::HardwareWidget(const uadf::Model& _uadf, const Signals& _sig)
{
  type_ = new qt::ComboBox();
  hardwares_ = new qt::StackedWidget();
  pwm_ = new PwmWidget(_uadf, _sig);
  dshot_ = new DShotWidget(_uadf, _sig);

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

void HardwareWidget::load(const YAML::Node& _node)
{
  type_->setCurrentText(_node[kTypeKey].as<QString>());
  setCurrentHardware(type_->currentIndex());

  for (int i = 0; i < hardwares_->count(); ++i) {
    const auto hardware = widget(i);
    hardware->load(_node[hardware->name()]);
  }

  pwm_->load(_node[kPwmLabel]);
  dshot_->load(_node[kDShotLabel]);
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

BaseHardwareWidget* HardwareWidget::widget(int _index)
{
  return qt::qPointerCast<BaseHardwareWidget>(hardwares_->widget(_index));
}

const BaseHardwareWidget* HardwareWidget::widget(int _index) const
{
  return qt::qConstPointerCast<BaseHardwareWidget>(hardwares_->widget(_index));
}

BaseHardwareWidget* HardwareWidget::selected()
{
  return widget(hardwares_->currentIndex());
}

const BaseHardwareWidget* HardwareWidget::selected() const
{
  return widget(hardwares_->currentIndex());
}

void HardwareWidget::setCurrentHardware(int _index)
{
  hardwares_->setCurrentIndex(_index);

  const auto hardware = widget(_index);
  pwm_->setNumChannels(hardware->numPwmChannels());
  dshot_->setNumChannels(hardware->numDShotChannels());
}

void HardwareWidget::onHardwareTypeChanged(int _index)
{
  qDebug().nospace() << "HardwareWidget::onHardwareTypeChanged(" << _index << ")";

  setCurrentHardware(_index);
}
}  // namespace tobas::gui::sa::hw
