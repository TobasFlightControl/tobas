#include <filesystem>
#include <QLabel>
#include <QVBoxLayout>
#include <QFileDialog>

#include <tobas_linux/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_setup_assistant/start/urdf_loader.hpp"
#include "tobas_setup_assistant/common.hpp"

namespace gui
{
namespace setup_assistant
{
URDFLoaderWidget::URDFLoaderWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot, SettingsWidget* settings)
  : node_(node), robot_(robot), settings_(settings), property_client_(node, tobas::kPropertyServerGCS, kPackageName)
{
  const auto rows = new QVBoxLayout(this);

  const auto label = new QLabel("Description Path");
  label->setFont(qt::DefaultFont(kLabelPSize, QFont::Bold));
  label->setAlignment(Qt::AlignTop);
  rows->addWidget(label);

  const auto instruction = new qt::DescriptionWidget(
    "Please set the urdf_path for the robot description and press the load button.", kBodyPSize);
  rows->addWidget(instruction);

  const auto cols = new QHBoxLayout();
  rows->addLayout(cols);

  file_text_ = new QLineEdit();
  file_text_->setReadOnly(true);
  file_text_->setFocusPolicy(Qt::NoFocus);
  cols->addWidget(file_text_);

  load_button_ = new QPushButton("Load");
  connect(load_button_, &QPushButton::clicked, this, &self::onLoadButtonClicked);
  cols->addWidget(load_button_);

  rows->addStretch();
}

void URDFLoaderWidget::onLoadButtonClicked()
{
  // 前回開いたパスを取得
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey, last_opened_dir) < 0)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    last_opened_dir = linux::homeDir();
  }

  // URDFのパスを取得
  const auto options = QFileDialog::DontUseNativeDialog;
  const auto urdf_path = QFileDialog::getOpenFileName(
    this, kTitle, QString::fromStdString(last_opened_dir), "Robot Description (*.urdf *.xacro)", nullptr, options);

  // キャンセルの場合は何もせずに終了 (そうしないと空文字が設定されてしまう)
  if (urdf_path.isEmpty())
    return;

  // パスをテキストに設定
  file_text_->setText(urdf_path);

  // ユーザが開いたディレクトリを保存
  const auto par_dir = std::filesystem::path(urdf_path.toStdString()).parent_path();
  if (property_client_.set(kLastOpenedDirKey, par_dir) < 0)
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
  if (property_client_.save() < 0)
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());

  // URDFをロード
  if (!robot_.loadFromPath(urdf_path.toStdString()))
  {
    qt::qErrorBox(this, "Failed to load robot description.");
    return;
  }

  // URDFを各ウィジェットに反映
  settings_->updateInternalDataStructures();

  qt::qInfoBox(this, "URDF is loaded successfully. Configure the settings for each tab.");
}
}  // namespace setup_assistant
}  // namespace gui
