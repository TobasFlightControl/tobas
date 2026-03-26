#include "tobas_qt_tools/widgets/ipv6_edit.hpp"

#include <QDebug>
#include <QHBoxLayout>
#include <QLabel>
#include <QRegularExpression>
#include <QRegularExpressionValidator>

#include <tobas_std_tools/check.hpp>

namespace tobas
{
namespace qt
{
IPv6Edit::IPv6Edit(QWidget* parent) : super(parent)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  const QRegularExpression hex4(R"(^[0-9A-Fa-f]{0,4}$)");
  for (int i = kNumFields - 1; i >= 0; --i) {
    fields_[i] = new QLineEdit();
    fields_[i]->setValidator(new QRegularExpressionValidator(hex4));

    cols->addWidget(fields_[i]);
    if (i != 0) {
      cols->addWidget(new QLabel(":"));
    }
  }
}

void IPv6Edit::clear()
{
  for (auto& field : fields_) {
    field->clear();
  }
}

bool IPv6Edit::isFilled() const
{
  for (auto& field : fields_) {
    if (field->text().isEmpty()) {
      return false;
    }
  }

  return true;
}

__uint128_t IPv6Edit::toInt() const
{
  __uint128_t res = 0;

  for (size_t i = 0; i < kNumFields; ++i) {
    const auto shift = 16 * i;
    const auto value = getFieldValue(i);
    res |= (static_cast<__uint128_t>(value) << shift);
  }

  return res;
}

QString IPv6Edit::toString() const
{
  QString res;

  for (int i = kNumFields - 1; i >= 0; --i) {
    const auto value = getFieldValue(i);
    res += QString::number(value, 16);
    if (i != 0) {
      res += ':';
    }
  }

  return res;
}

void IPv6Edit::setFromInt(__uint128_t address)
{
  for (size_t i = 0; i < kNumFields; ++i) {
    const auto shift = 16 * i;
    const auto value = static_cast<uint16_t>((address >> shift) & 0xFFFF);
    setFieldValue(i, value);
  }
}

uint16_t IPv6Edit::getFieldValue(size_t idx) const
{
  const auto text = fields_.at(idx)->text();

  if (text.isEmpty()) {
    qWarning() << "Field" << idx << "is empty.";
    return 0;
  }

  bool ok = false;
  const auto value = text.toInt(&ok, 16);
  TOBAS_CHECK(ok);
  TOBAS_CHECK(0 <= value && value <= UINT16_MAX);
  return static_cast<uint16_t>(value);
}

void IPv6Edit::setFieldValue(size_t idx, uint16_t value)
{
  fields_.at(idx)->setText(QString::number(value, 16).rightJustified(4, '0'));
}
}  // namespace qt
}  // namespace tobas
