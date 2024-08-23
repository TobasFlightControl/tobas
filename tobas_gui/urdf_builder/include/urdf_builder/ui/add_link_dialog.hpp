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
class URDFBuilderPanel;

namespace ui
{
class AddLinkDialog : public QDialog
{
  Q_OBJECT

public:
  explicit AddLinkDialog(URDFBuilderPanel* main, const QStringList& link_names, view_model::LinkViewModel& link_vm);

private Q_SLOTS:
  void LinkNameLineEditTextChanged(const QString& text);
  void JointNameLineEditTextChanged(const QString& text);
  void JointParentComboBoxIndexChanged(int index);

private:
  URDFBuilderPanel* main_;
  Ui::AddLinkDialogUIPtr ui_;
  view_model::LinkViewModel& link_vm_;

  void checkValidity();
  void enableOkButton(bool enable);
};
}  // namespace ui
}  // namespace urdf_builder
