#pragma once

#include <QIcon>
#include <QLineEdit>

namespace qt
{
class PasswordEdit : public QLineEdit
{
  Q_OBJECT

  using self = PasswordEdit;
  using super = QLineEdit;

public:
  explicit PasswordEdit(QWidget* parent = nullptr);

private:
  QIcon eye_on_;
  QIcon eye_off_;

  QAction* toggle_;

  void setMode(bool on);

private Q_SLOTS:
  void onIconToggled(bool on);
};
}  // namespace qt
