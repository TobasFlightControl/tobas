#pragma once

#include <QFileDialog>
#include <QLineEdit>
#include <QPushButton>

#include <tobas_qt_tools/widgets/label.hpp>

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
  QLineEdit* file_name_;
  qt::Label* warn_text_;

private Q_SLOTS:
  void onFilePathChanged();
};
}  // namespace sa
}  // namespace gui
