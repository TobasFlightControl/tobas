#pragma once

#include <filesystem>
#include <QCheckBox>
#include <QHBoxLayout>

namespace gui
{
namespace sim
{
class WorldWidget_Base : public QWidget
{
  Q_OBJECT

public:
  QCheckBox* checkbox;

  explicit WorldWidget_Base(const QString& label);

  virtual std::filesystem::path worldPath() const = 0;

  bool isChecked() const;
  void setChecked(bool checked);

protected:
  QHBoxLayout* cols_;

protected Q_SLOTS:
  virtual void onCheckBoxToggled(bool toggled) = 0;
};
}  // namespace sim
}  // namespace gui
