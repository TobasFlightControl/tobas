#pragma once

#include <QIntValidator>

namespace qt
{
/**
 * ===== QFormLayoutとの違い =====
 * - validate() で Intermediate を正確に処理
 */
class IntValidator : public QIntValidator
{
  Q_OBJECT

  using super = QIntValidator;

public:
  using super::QIntValidator;

  State validate(QString& input, int& pos) const override;
};
}  // namespace qt
