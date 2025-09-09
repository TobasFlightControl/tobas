#pragma once

#include <array>

#include <QLineEdit>
#include <QWidget>

namespace qt
{
class IPv4Edit : public QWidget
{
  Q_OBJECT

  using super = QWidget;

  static constexpr size_t kNumFields = 4;

public:
  explicit IPv4Edit(QWidget* parent = nullptr);

  void clear();

  bool isFilled() const;

  uint32_t toInt() const;

  /* xxx.xxx.xxx.xxx */
  QString toString() const;

  void setFromInt(uint32_t address);

private:
  std::array<QLineEdit*, kNumFields> fields_;

  uint8_t getFieldValue(size_t idx) const;
  void setFieldValue(size_t idx, uint8_t value);
};
}  // namespace qt
