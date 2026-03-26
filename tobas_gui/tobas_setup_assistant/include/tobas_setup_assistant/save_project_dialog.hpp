#pragma once

#include <QFileDialog>
#include <QLineEdit>
#include <QPushButton>

#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
class SaveProjectDialog : public QFileDialog
{
  Q_OBJECT

  using self = SaveProjectDialog;
  using super = QFileDialog;

public:
  explicit SaveProjectDialog(QWidget* parent, const QString& dir, const QString& dflt_name);

protected:
  bool eventFilter(QObject* obj, QEvent* event) override;

private:
  QPushButton* save_button_;
  QLineEdit* proj_name_;
  tobas::qt::Label* warn_text_;

private Q_SLOTS:
  void onProjectPathChanged();
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
