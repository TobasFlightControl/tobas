#pragma once

#include <memory>
#include <QtWidgets/QtWidgets>

#include "../view_model/link_view_model.hpp"

namespace Ui
{
class AddLinkDialogUI;

using AddLinkDialogUIPtr = std::shared_ptr<AddLinkDialogUI>;
}  // namespace Ui

namespace urdf_builder
{
namespace ui
{
class AddLinkDialog : public QDialog
{
  Q_OBJECT

public:
  explicit AddLinkDialog(const QStringList& link_names, view_model::LinkViewModel& link_vm);

private Q_SLOTS:
  void LinkNameLineEditTextChanged(const QString& text);
  void JointNameLineEditTextChanged(const QString& text);
  void JointParentComboBoxIndexChanged(int index);

private:
  Ui::AddLinkDialogUIPtr ui_;
  view_model::LinkViewModel& link_vm_;
};
}  // namespace ui
}  // namespace urdf_builder
