#pragma once

#include <filesystem>

#include <QHBoxLayout>
#include <QRadioButton>

namespace gui
{
namespace sim
{
class WorldWidget_Base : public QWidget
{
  Q_OBJECT

public:
  QRadioButton* radio_button;

  explicit WorldWidget_Base(const QString& label);

  virtual std::filesystem::path worldPath() const = 0;
  virtual void setContentsEnabled(bool enable) = 0;

  bool isChecked() const;
  void setChecked(bool checked);

protected:
  QHBoxLayout* cols_;

private Q_SLOTS:

  void onRadioButtonToggled(bool checked);
};
}  // namespace sim
}  // namespace gui
