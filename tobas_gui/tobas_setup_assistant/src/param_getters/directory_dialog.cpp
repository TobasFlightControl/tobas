#include "tobas_setup_assistant/param_getters/directory_dialog.hpp"

#include <filesystem>

#include <rcutils/env.h>
#include <QHBoxLayout>
#include <QPushButton>

#include <tobas_constants/constants.hpp>
#include <tobas_string_tools/core.hpp>

using namespace std;

namespace gui
{
namespace sa
{
ParamGetterWidget_DirDialog::ParamGetterWidget_DirDialog(
  rclcpp::Node::SharedPtr node,
  const QString& param_name,
  const QString& description_text)
  : super(param_name, description_text)
  , node_(node)
  , last_opend_dir_key_("last_opened_dir/dir_dialog/" + str::replace(param_name.toStdString(), " ", "_"))
  , property_client_(node, tobas::kPropertyServerName, kPackageName)
{
  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  path_ = new QLineEdit();
  path_->setReadOnly(true);
  path_->setFocusPolicy(Qt::NoFocus);
  cols->addWidget(path_);

  const auto browse_button = new QPushButton("Browse");
  cols->addWidget(browse_button);

  connect(path_, &QLineEdit::textChanged, this, &self::onTextChanged);
  connect(browse_button, &QPushButton::clicked, this, &self::onBrowseButtonClicked);
}

QString ParamGetterWidget_DirDialog::getValue() const
{
  return path_->text();
}

bool ParamGetterWidget_DirDialog::setValue(const QString& src)
{
  path_->setText(src);
  return true;
}

void ParamGetterWidget_DirDialog::onTextChanged(const QString& text)
{
  Q_EMIT pathChanged(text);
}

void ParamGetterWidget_DirDialog::onBrowseButtonClicked()
{
  string last_opened_dir;
  if (property_client_.get(last_opend_dir_key_, last_opened_dir) < 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    last_opened_dir = rcutils_get_home_dir();
  }

  const auto options = QFileDialog::DontUseNativeDialog | QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks;
  const auto path = QFileDialog::getExistingDirectory(this, kTitle, QString::fromStdString(last_opened_dir), options);
  if (path.isEmpty())  // Cancelの場合
  {
    return;
  }

  path_->setText(path);

  // 最後に開かれたパスを保存
  const auto par_dir = filesystem::path(path.toStdString()).parent_path();
  if (property_client_.set(last_opend_dir_key_, par_dir) < 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    return;
  }
  if (property_client_.save() < 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    return;
  }
}
}  // namespace sa
}  // namespace gui
