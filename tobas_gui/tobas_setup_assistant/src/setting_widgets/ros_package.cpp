#include <filesystem>

#include <tobas_linux/core.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/ros_package.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace setup_assistant
{
ROSPackageWidget::ROSPackageWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot) : node_(node), robot_(robot)
{
}

const char* ROSPackageWidget::name() const
{
  return "ROS Package";
}

const char* ROSPackageWidget::title() const
{
  return "Generate ROS Package";
}

const char* ROSPackageWidget::description() const
{
  return "Based on the previous settings, we will generate the necessary ROS packages for using Tobas. "
         "Please specify the path for the package and click the \"Generate\" button.";
}

void ROSPackageWidget::onInit()
{
  pardir_ = new ParamGetterWidget_DirDialog(node_, "Parent Directory", "");
  connect(pardir_, &ParamGetterWidget_DirDialog::pathChanged, this, &self::onPathChanged);
  addWidget(pardir_);

  tbs_name_ = new ParamGetterWidget_LineEdit("Package Name", "");
  connect(tbs_name_, &ParamGetterWidget_LineEdit::textChanged, this, &self::onPathChanged);
  addWidget(tbs_name_);

  const auto text = new QLabel("The package will be generated as");
  text->setFont(qt::DefaultFont(kBodyPSize));
  text->setFixedHeight(kTextHeight);
  setAlignment(Qt::AlignTop);
  addWidget(text);

  tbs_path_ = new QLabel();
  tbs_path_->setFont(qt::DefaultFont(kBodyPSize, QFont::Bold));
  tbs_path_->setFixedHeight(kTextHeight);
  tbs_path_->setAlignment(Qt::AlignTop);
  addWidget(tbs_path_);

  generate_button_ = new QPushButton("Generate");
  generate_button_->setFixedSize(kButtonWidth, kButtonHeight);
  generate_button_->setEnabled(false);
  connect(generate_button_, &QPushButton::clicked, this, &self::onGenerateButtonClicked);
  addWidgetCenter(generate_button_);
}

void ROSPackageWidget::onOpened()
{
  return;
}

void ROSPackageWidget::updateInternalDataStructures()
{
  // デフォルトの親ディレクトリを設定
  const auto default_pardir = path::join(linux::expandUser(tobas::kColconWSPath), "src");
  pardir_->setValue(QString::fromStdString(default_pardir));

  // デフォルトの親ディレクトリが存在しなければ作成
  if (!fs::exists(default_pardir))
    if (!fs::create_directories(default_pardir))
      qt::qErrorBox(this, QString::fromStdString("Failed to create \"" + default_pardir + "\"."));

  // デフォルトのTBSパッケージ名を設定
  const auto tbs_name = "tobas_" + robot_.robotName();
  tbs_name_->setValue(QString::fromStdString(tbs_name));
}

bool ROSPackageWidget::isValid()
{
  const auto pardir = pardir_->getValue();
  const auto tbs_name = tbs_name_->getValue();
  const auto tbs_path = tbs_path_->text();

  // 親ディレクトリが存在することを確認
  if (!fs::is_directory(pardir.toStdString()))
  {
    qt::qErrorBox(this, pardir + " does not exist.");
    return false;
  }

  // パッケージ名が無効な文字を含んでいないことを確認
  if (tbs_name.contains('/') || tbs_name.contains('.') || tbs_name.contains(' '))
  {
    qt::qErrorBox(this, "Invalid package name: " + tbs_name);
    return false;
  }

  // パッケージパスが既に存在する場合は置換するかどうかをユーザに確認
  if (fs::exists(tbs_path.toStdString()))
    if (!qt::yesOrNo(this, tbs_path + " already exists. Do you want to replace it?", qt::QMessageLevel::WARN))
      return false;

  return true;
}

YAML::Node ROSPackageWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[pardir_->name()] = pardir_->getValue();
  node[tbs_name_->name()] = tbs_name_->getValue();

  return node;
}

void ROSPackageWidget::load(const YAML::Node& node)
{
  pardir_->setValue(node[pardir_->name()].as<QString>());
  tbs_name_->setValue(node[tbs_name_->name()].as<QString>());
}

QString ROSPackageWidget::tbsName() const
{
  return tbs_name_->getValue();
}

QString ROSPackageWidget::tbsPath() const
{
  return tbs_path_->text();
}

void ROSPackageWidget::onPathChanged()
{
  const auto pardir = pardir_->getValue().toStdString();
  const auto tbs_name = tbs_name_->getValue().toStdString();

  const auto path = fs::path(pardir) / (tbs_name + tobas::kTBSExtension);
  tbs_path_->setText(QString::fromStdString(path));

  generate_button_->setEnabled(!pardir.empty() && !tbs_name.empty());
}

void ROSPackageWidget::onGenerateButtonClicked()
{
  Q_EMIT generateButtonClicked();
}
}  // namespace setup_assistant
}  // namespace gui
