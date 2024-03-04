#pragma once

#include <QtWidgets>

namespace tobas_gcs
{
class IpAddressGetter : public QWidget
{
  Q_OBJECT

public:
  explicit IpAddressGetter(QWidget* parent);

  QString getIP() const;
  void setIP(uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4);

private:
  QSpinBox* spin_boxes_[4];

Q_SIGNALS:
  void ipChanged(const QString& ip);

private Q_SLOTS:
  void onValueChanged(int);
};
}  // namespace tobas_gcs
