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
class ParamGetterWidget_FileDialog : public ParamGetterWidget<QString>
{
  Q_OBJECT

  using self = ParamGetterWidget_FileDialog;
  using super = ParamGetterWidget<QString>;

Q_SIGNALS:
  void pathChanged(const QString& text);

public:
  explicit ParamGetterWidget_FileDialog(
    rclcpp::Node::SharedPtr node,
    const QString& param_name,
    const QString& description_text);

  QString getValue() const override;
  bool setValue(const QString& src) override;

  void setInitialFilter(const QString& init_filter);

private Q_SLOTS:
  void onTextChanged(const QString& text);
  void onBrowseButtonClicked();

private:
  const rclcpp::Node::SharedPtr node_;
  const std::string last_opend_dir_key_;
  QString init_filter_ = "All (*)";
  ptree::PropertyClient property_client_;
  QLineEdit* path_;
};
}  // namespace sa
}  // namespace gui
