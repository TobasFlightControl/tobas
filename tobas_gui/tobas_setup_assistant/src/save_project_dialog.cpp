#include "tobas_setup_assistant/save_project_dialog.hpp"

#include <QDialogButtonBox>
#include <QEvent>
#include <QGridLayout>
#include <QKeyEvent>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/path.hpp>
#include <tobas_ros2_tools/package.hpp>
#include <tobas_ros2_tools/util.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
SaveProjectDialog::SaveProjectDialog(QWidget* parent, const QString& dir, const QString& dflt_name)
  : super(parent, "Save Tobas Project", dir)
{
  setOptions(ShowDirsOnly | DontUseNativeDialog);  // カスタム設定のためにQtのダイアログを使用
  setAcceptMode(AcceptSave);  // setFileMode(Directory)を実行するとSaveボタンが消えることに注意
  setFilter(QDir::AllDirs | QDir::Hidden | QDir::NoDotAndDotDot);
  setLabelText(FileName, "Project name:");
  setDefaultSuffix("TBS");
  selectFile(dflt_name);

  // 保存ボタンを取得
  const auto button_box = findChild<QDialogButtonBox*>("buttonBox");
  save_button_ = button_box->button(QDialogButtonBox::Save);

  // ファイル名を取得
  proj_name_ = findChild<QLineEdit*>("fileNameEdit");
  proj_name_->installEventFilter(this);

  // 警告文をレイアウトの一番下に挿入
  warn_text_ = new tobas::qt::Label();
  warn_text_->setTextColor(Qt::red);
  const auto grid = tobas::qt::qPointerCast<QGridLayout>(layout());
  grid->addWidget(warn_text_, grid->rowCount(), 0, 1, grid->columnCount());

  // パスが変わったらその都度保存可能性をチェック
  connect(proj_name_, &QLineEdit::textChanged, this, &self::onProjectPathChanged);
  connect(this, &super::directoryEntered, this, &self::onProjectPathChanged);

  // 最初のチェック
  onProjectPathChanged();
}

bool SaveProjectDialog::eventFilter(QObject* obj, QEvent* event)
{
  if (obj == proj_name_) {
    // 保存ボタンが有効化されている場合のみ認める
    if (event->type() == QEvent::KeyPress) {
      const auto key_event = static_cast<QKeyEvent*>(event);
      if (key_event->key() == Qt::Key_Return || key_event->key() == Qt::Key_Enter) {
        if (save_button_->isEnabled()) {
          accept();
        }
      }
    }
  }

  return super::eventFilter(obj, event);
}

void SaveProjectDialog::onProjectPathChanged()
{
  const auto dir = directory().absolutePath();
  const auto proj_name = proj_name_->text();

  // ホームディレクトリ以下でなければならない
  if (!dir.startsWith(ros2::getHomeDir())) {
    warn_text_->setText("The Tobas project must be located under your home directory.");
    save_button_->setEnabled(false);
    return;
  }

  // srcディレクトリ以下でなければならない
  if (!dir.contains("/src/") && !dir.endsWith("/src")) {
    warn_text_->setText("The Tobas project must be located under a \"src\" directory.");
    save_button_->setEnabled(false);
    return;
  }

  // プロジェクト内にプロジェクトを作ることはできない
  if (dir.contains(cmn::kProjectExtension + QString("/")) || dir.endsWith(cmn::kProjectExtension)) {
    warn_text_->setText("A project cannot be created inside another project.");
    save_button_->setEnabled(false);
    return;
  }

  // ファイル名が設定されていなければならない
  if (proj_name.isEmpty()) {
    warn_text_->setText("Please specify a project name.");
    save_button_->setEnabled(false);
    return;
  }

  // 拡張子を除いたパッケージ名がROSの慣習に沿っていなければならない
  const auto pkg_name = QFileInfo(proj_name).completeBaseName();
  if (!ros2::isValidPackageName(pkg_name.toStdString())) {
    warn_text_->setText("Project name is invalid. It must match: ^[a-z][a-z0-9_]*$");
    save_button_->setEnabled(false);
    return;
  }

  // 拡張子が設定されている場合は決められた拡張子でなければならない
  if (proj_name.contains('.')) {
    if (!proj_name.endsWith(cmn::kProjectExtension)) {
      warn_text_->setText("Invalid project extension.");
      save_button_->setEnabled(false);
      return;
    }

    if (tobas::qt::getBaseName(proj_name).isEmpty()) {
      warn_text_->setText("The base name of the project is empty.");
      save_button_->setEnabled(false);
      return;
    }
  }

  warn_text_->clear();
  save_button_->setEnabled(true);
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
