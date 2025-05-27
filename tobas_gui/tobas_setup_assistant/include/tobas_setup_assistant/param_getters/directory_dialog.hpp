#pragma once

#include <QFileDialog>
#include <QLineEdit>

#include <tobas_constants/constants.hpp>
#include <tobas_property_client/property_client.hpp>

#include "./base.hpp"

namespace gui
{
namespace sa
{
class ParamGetterWidget_DirDialog : public ParamGetterWidget<QString>
{
  Q_OBJECT

  using self = ParamGetterWidget_DirDialog;
  using super = ParamGetterWidget<QString>;

Q_SIGNALS:
  void pathChanged(const QString& text);

public:
  explicit ParamGetterWidget_DirDialog(
    rclcpp::Node::SharedPtr node,
    const QString& param_name,
    const QString& description_text);

  QString getValue() const override;
  bool setValue(const QString& src) override;

private Q_SLOTS:
  void onTextChanged(const QString& text);
  void onBrowseButtonClicked();

private:
  const rclcpp::Node::SharedPtr node_;
  const std::string last_opend_dir_key_;
  ptree::PropertyClient property_client_;
  QLineEdit* path_;
};
}  // namespace sa
}  // namespace gui
