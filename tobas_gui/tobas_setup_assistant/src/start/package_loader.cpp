#include <filesystem>
#include <rcutils/env.h>
#include <QLabel>
#include <QVBoxLayout>
#include <QFileDialog>

#include <tobas_yaml_tools/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_gui_common/package.hpp>

#include "tobas_setup_assistant/start/package_loader.hpp"
#include "tobas_setup_assistant/common.hpp"

namespace gui
{
namespace setup_assistant
{
PackageLoaderWidget::PackageLoaderWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot, SettingsWidget* settings)
  : node_(node), robot_(robot), settings_(settings), property_client_(node, tobas::kPropertyServerName, kPackageName)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto label = new QLabel("Tobas Configuration Package Path");
  label->setFont(qt::DefaultFont(kLabelPSize, QFont::Bold));
  label->setAlignment(Qt::AlignTop);
  rows->addWidget(label);

  const auto instruction = new qt::DescriptionWidget(
    "Please set the path for the Tobas configuration package and press the load button.", kBodyPSize);
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

void PackageLoaderWidget::onLoadButtonClicked()
{
  // 前回開いたパスを取得
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey, last_opened_dir) < 0)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    last_opened_dir = rcutils_get_home_dir();
  }

  // Tobasパッケージのパスを取得
  const auto options = QFileDialog::DontUseNativeDialog | QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks;
  const auto tbs_path =
    QFileDialog::getExistingDirectory(this, kTitle, QString::fromStdString(last_opened_dir), options);

  // キャンセルの場合は何もせずに終了 (そうしないと空文字が設定されてしまう)
  if (tbs_path.isEmpty())
    return;

  // 拡張子をチェック
  if (!tbs_path.endsWith(common::kTBSExtension))
  {
    qt::qErrorBox(this, "\"" + tbs_path + "\" is not a Tobas configuration package (*" + common::kTBSExtension + ").");
    return;
  }

  // URDFの存在を確認
  const auto urdf_path = common::getOriginalURDFPath(tbs_path.toStdString());
  if (!std::filesystem::is_regular_file(urdf_path))
  {
    qt::qErrorBox(
      this,
      "\"" + QString::fromStdString(urdf_path) + "\" does not exist. Please create a new Tobas configuration package.");
    return;
  }

  // ユーザ設定ファイルの存在を確認
  const auto settings_path = common::getSettingsPath(tbs_path.toStdString());
  if (!std::filesystem::is_regular_file(settings_path))
  {
    qt::qErrorBox(
      this, "\"" + QString::fromStdString(settings_path)
              + "\" does not exist. Please create a new Tobas configuration package.");
    return;
  }

  // パスをテキストに設定
  file_text_->setText(tbs_path);

  // ユーザが開いたディレクトリを保存
  const auto par_dir = std::filesystem::path(tbs_path.toStdString()).parent_path();
  if (property_client_.set(kLastOpenedDirKey, par_dir) < 0)
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
  if (property_client_.save() < 0)
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());

  // URDFをロード
  if (!robot_.loadFromPath(urdf_path))
  {
    qt::qErrorBox(this, "Failed to load robot description.");
    return;
  }

  // URDFを各ウィジェットに反映
  settings_->updateInternalDataStructures();

  // ユーザ設定を読み込む
  YAML::Node node;
  if (!yaml::load(settings_path, node))
  {
    qt::qErrorBox(this, "The user configuration file is collapsed. Please create a new Tobas configuration package.");
    return;
  }
  if (!settings_->load(node))
    return;

  qt::qInfoBox(this, "Tobas configuration package is loaded successfully.");
}
}  // namespace setup_assistant
}  // namespace gui
