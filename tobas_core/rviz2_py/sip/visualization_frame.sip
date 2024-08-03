namespace rviz_common
{
// class VisualizationFramePy
// {
class VisualizationFramePy: QMainWindow /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz_common"
#include "../include/rviz2_py/visualization_frame_py.hpp"
%End

public:
  VisualizationFramePy(QWidget* parent = nullptr);
  ~VisualizationFramePy();

  void initialize(const QString& display_config_file = "");

  // rviz_rendering::RenderWindow* getRenderWindow();
  void setApp(QApplication* app);
  void setHelpPath(const QString& help_path);
  void setSplashPath(const QString& splash_path);
  void setDisplayTitleFormat(const QString& title_format);
  VisualizationManager* getManager();
  QWidget* getParentWindow();
  // PanelDockWidget* addPane(
  //   const QString& name,
  //   QWidget* panel,
  //   Qt::DockWidgetArea area = Qt::LeftDockWidgetArea,
  //   bool floating = true) override;
  void loadPersistentSettings();
  void savePersistentSettings();
  void loadDisplayConfig(const QString& path);
  bool saveDisplayConfig(const QString& path);
  QString getErrorMessage() const;
  void load(const Config& config);
  void save(Config config);
  void setHideButtonVisibility(bool visible);

public Q_SLOTS:
  void setDisplayConfigModified();
  void setStatus(const QString& message);
  void setFullScreen(bool full_screen);
  void exitFullScreen();

Q_SIGNALS:
  void statusUpdate(const QString& message);
  void fullScreenChange(bool hidden);
};
};  // namespace rviz_common
