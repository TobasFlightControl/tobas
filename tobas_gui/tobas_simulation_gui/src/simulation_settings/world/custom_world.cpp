#include "tobas_simulation_gui/simulation_settings/world/custom_world.hpp"

#include <rcutils/env.h>
#include <QFileDialog>

#include <tobas_ros2_tools/path.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_simulation_gui/constants.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sim
{
WorldWidget_Custom::WorldWidget_Custom(rclcpp::Node::SharedPtr node)
  : super("Custom World"), node_(node), property_client_(node, tobas::kPropertyServerName, kPackageName)
{
  file_text_ = new QLineEdit();
  file_text_->setReadOnly(true);
  file_text_->setFocusPolicy(Qt::NoFocus);
  cols_->addWidget(file_text_);

  browse_button_ = new QPushButton("Browse");
  connect(browse_button_, &QPushButton::clicked, this, &self::onBrowseButtonClicked);
  cols_->addWidget(browse_button_);
}

fs::path WorldWidget_Custom::worldPath() const
{
  return file_text_->text().toStdString();
}

void WorldWidget_Custom::setContentsEnabled(bool enable)
{
  file_text_->setEnabled(enable);
  browse_button_->setEnabled(enable);
}

void WorldWidget_Custom::onBrowseButtonClicked()
{
  // 前回開いたパスを取得
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey, last_opened_dir) < 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    last_opened_dir = rcutils_get_home_dir();
  }

  // worldのパスを取得
  const auto options = QFileDialog::DontUseNativeDialog;
  const auto file_path = QFileDialog::getOpenFileName(
    this, "Select World File", QString::fromStdString(last_opened_dir), "Gazebo World (*.world)", nullptr, options);

  // キャンセルの場合は何もせずに終了
  if (file_path.isEmpty()) {
    return;
  }

  // パスをテキストに設定
  file_text_->setText(file_path);

  // ユーザが開いたディレクトリを保存
  const auto par_dir = std::filesystem::path(file_path.toStdString()).parent_path();
  if (property_client_.set(kLastOpenedDirKey, par_dir) < 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
  }
  if (property_client_.save() < 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
  }
}
}  // namespace sim
}  // namespace gui
