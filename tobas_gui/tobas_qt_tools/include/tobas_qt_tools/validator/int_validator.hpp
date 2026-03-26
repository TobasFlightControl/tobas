#pragma once

#include <QIntValidator>

namespace tobas
{
namespace qt
{
/**
 * ===== QFormLayout との違い =====
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
}  // namespace tobas
