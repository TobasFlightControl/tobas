#include "tobas_simulation_gui/simulation_settings/world/custom_world.hpp"

#include <QFileDialog>
#include <QHBoxLayout>

#include <tobas_ros2_tools/path.hpp>
#include <tobas_ros2_tools/util.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace sim
{
CustomWorldWidget::CustomWorldWidget(rclcpp::Node::SharedPtr node)
  : node_(node), property_client_(node, "tobas_simulation_gui/simulation_settings/world/custom_world")
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  file_text_ = new QLineEdit();
  file_text_->setReadOnly(true);
  file_text_->setFocusPolicy(Qt::NoFocus);
  cols->addWidget(file_text_);

  browse_button_ = new QPushButton("Browse");
  connect(browse_button_, &QPushButton::clicked, this, &self::onBrowseButtonClicked);
  cols->addWidget(browse_button_);
}

fs::path CustomWorldWidget::worldPath() const
{
  return file_text_->text().toStdString();
}

void CustomWorldWidget::onBrowseButtonClicked()
{
  // 前回開いたパスを取得
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey, last_opened_dir) < 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    last_opened_dir = ros2::getHomeDir();
  }

  // worldのパスを取得
  const auto file_path = QFileDialog::getOpenFileName(
    this,
    "Select World File",
    QString::fromStdString(last_opened_dir),
    "Gazebo World (*.world)",
    nullptr,
    QFileDialog::DontUseNativeDialog);

  // キャンセルの場合は何もせずに終了
  if (file_path.isEmpty()) {
    return;
  }

  // パスをテキストに設定
  file_text_->setText(file_path);

  // ユーザが開いたディレクトリを保存
  const auto par_dir = fs::path(file_path.toStdString()).parent_path();
  if (property_client_.set(kLastOpenedDirKey, par_dir) < 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
  }
  if (property_client_.save() < 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
  }
}
}  // namespace sim
}  // namespace gui
