#include <filesystem>
#include <QPushButton>
#include <QHBoxLayout>

#include <tobas_std_tools/string.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_setup_assistant/param_getters/file_dialog.hpp"

using namespace std;

namespace gui
{
namespace setup_assistant
{
ParamGetterWidget_FileDialog::ParamGetterWidget_FileDialog(
  rclcpp::Node::SharedPtr node,
  const QString& param_name,
  const QString& description_text,
  const QString& _default,
  const QString& initial_filter)
  : super(param_name, description_text),
    node_(node),
    last_opend_dir_key_("last_opened_dir/file_dialog/" + tobas_std::replace(param_name.toStdString(), " ", "_")),
    init_filter_(initial_filter),
    property_client_(node, tobas::kPropertyServerGCS, kPackageName)
{
  auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  path_ = new QLineEdit(_default);
  path_->setReadOnly(true);
  path_->setFocusPolicy(Qt::NoFocus);
  cols->addWidget(path_);

  auto browse_button = new QPushButton("Browse");
  cols->addWidget(browse_button);

  connect(path_, SIGNAL(QLineEdit::textChanged(const QString&)), this, SLOT(onTextChanged(const QString&)));
  connect(browse_button, SIGNAL(QPushButton::clicked()), this, SLOT(onBrowseButtonClicked()));
}

QString ParamGetterWidget_FileDialog::get() const
{
  return path_->text();
}

bool ParamGetterWidget_FileDialog::set(const QString& src)
{
  path_->setText(src);
  return true;
}

void ParamGetterWidget_FileDialog::onTextChanged(const QString& text)
{
  Q_EMIT pathChanged(text);
}

void ParamGetterWidget_FileDialog::onBrowseButtonClicked()
{
  string last_opened_dir;
  if (property_client_.get(last_opend_dir_key_, last_opened_dir) < 0)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    last_opened_dir = linux::homeDir();
  }

  const auto options = QFileDialog::DontUseNativeDialog;
  const auto path =
    QFileDialog::getOpenFileName(this, kTitle, QString::fromStdString(last_opened_dir), init_filter_, nullptr, options);
  if (path.isEmpty())  // Cancelの場合
    return;

  path_->setText(path);

  // 最後に開かれたパスを保存
  const auto par_dir = filesystem::path(path.toStdString()).parent_path();
  if (property_client_.set(last_opend_dir_key_, par_dir) < 0)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    return;
  }
  if (property_client_.save() < 0)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    return;
  }
}
}  // namespace setup_assistant
}  // namespace gui
