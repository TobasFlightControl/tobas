#pragma once

#include <eigen3/Eigen/Core>
#include <QFileDialog>

#include <tobas_property_client/property_client.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class ParamGetterWidget_DoubleTable : public ParamGetterWidget<Eigen::MatrixXd>
{
  Q_OBJECT

  using self = ParamGetterWidget_DoubleTable;
  using super = ParamGetterWidget<Eigen::MatrixXd>;

  static constexpr int kButtonHeight = 30;
  static constexpr int kButtonWidth = 100;
  static constexpr double kDefaultValue = 0.;
  static constexpr int kDefaultDecimals = 2;

Q_SIGNALS:
  void dataChanged();

public:
  explicit ParamGetterWidget_DoubleTable(
    rclcpp::Node::SharedPtr node,
    const QString& param_name,
    const QStringList& labels,
    const QString& description_text);

  Eigen::MatrixXd getValue() const override;
  bool setValue(const Eigen::MatrixXd& src) override;

  qt::TableWidget* table();

  void setDecimals(const QVector<int>& decimals);
  void setMinimum(const QVector<double>& minimum);
  void setMaximum(const QVector<double>& maximum);
  void setDefault(const QVector<double>& _default);
  void setSuffix(const QVector<QString>& suffix);

  int count() const;

private Q_SLOTS:
  void addRow();
  void deleteRow();
  void loadCSV();
  void onCellValueChanged();

private:
  const rclcpp::Node::SharedPtr node_;
  const std::string last_opend_dir_key_;
  const QStringList labels_;
  const int num_entry_;

  QVector<double> minimum_;
  QVector<double> maximum_;
  QVector<double> default_;
  QVector<int> decimals_;
  QVector<QString> suffix_;

  qt::TableWidget* table_;

  ptree::PropertyClient property_client_;

  QString getCSVFilePath();
  bool isValidData(const Eigen::MatrixXd& src);
  void saveLastOpenedDir(const std::string& dir);
};
}  // namespace setup_assistant
}  // namespace gui
