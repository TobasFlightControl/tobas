#include <ros/ros.h>

#include "../../include/tobas_gcs/utils/ip_getter.hpp"

namespace tobas_gcs
{
IpAddressGetter::IpAddressGetter(QWidget* parent) : QWidget(parent)
{
  auto* cols = new QHBoxLayout(this);
  setLayout(cols);

  // IPアドレスの各オクテット用のQSpinBoxを作成
  for (size_t i = 0; i < 4; ++i)
  {
    spin_boxes_[i] = new QSpinBox(this);
    spin_boxes_[i]->setRange(0, 255);  // IPアドレスのオクテットは0から255の範囲
    cols->addWidget(spin_boxes_[i]);

    // 各QSpinBoxの間にドットを挿入
    if (i < 3)
      cols->addWidget(new QLabel(".", this));

    connect(spin_boxes_[i], SIGNAL(valueChanged(int)), this, SLOT(onValueChanged(int)));
  }
}

QString IpAddressGetter::getIP() const
{
  QString ip;
  for (size_t i = 0; i < 4; ++i)
  {
    if (i > 0)
      ip += ".";
    ip += QString::number(spin_boxes_[i]->value());
  }
  return ip;
}

void IpAddressGetter::setIP(uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4)
{
  spin_boxes_[0]->setValue(x1);
  spin_boxes_[1]->setValue(x2);
  spin_boxes_[2]->setValue(x3);
  spin_boxes_[3]->setValue(x4);
}

void IpAddressGetter::onValueChanged(int)
{
  ROS_INFO_STREAM("IpAddressGetter::onValueChanged");

  Q_EMIT ipChanged(getIP());
}
}  // namespace tobas_gcs
