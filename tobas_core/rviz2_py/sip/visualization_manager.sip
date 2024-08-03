namespace rviz_common
{
// class VisualizationManager
// {
class VisualizationManager: QObject /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz_common"
#include <rviz_common/visualization_manager.hpp>
%End

public:
  // VisualizationManager(
  //   RenderPanel* render_panel,
  //   ros_integration::RosNodeAbstractionIface::WeakPtr ros_node_abstraction,
  //   WindowManagerInterface* wm,
  //   rclcpp::Clock::SharedPtr clock);
  ~VisualizationManager();

  void initialize();
  void startUpdate();
  void stopUpdate();
  Display* createDisplay(const QString& class_lookup_name, const QString& name, bool enabled);
  void addDisplay(Display* display, bool enabled);
  void removeAllDisplays();
  void load(const Config& config);
  void save(Config config) const;
  QString getFixedFrame() const;
  void setFixedFrame(const QString& frame);
  Ogre::SceneManager* getSceneManager() const;
  // RenderPanel* getRenderPanel() const;
  double getWallClock();
  double getROSTime();
  double getWallClockElapsed();
  double getROSTimeElapsed();
  // void handleChar(QKeyEvent* event, RenderPanel* panel);
  // void handleMouseEvent(const ViewportMouseEvent& event);
  // std::shared_ptr<rviz_common::interaction::HandlerManagerIface> getHandlerManager() const;
  // std::shared_ptr<rviz_common::interaction::SelectionManagerIface> getSelectionManager() const;
  // std::shared_ptr<rviz_common::interaction::ViewPickerIface> getViewPicker() const;
  // ToolManager* getToolManager() const;
  // ViewManager* getViewManager() const;
  // rviz_common::transformation::TransformationManager* getTransformationManager();
  void lockRender();
  void unlockRender();
  void queueRender();
  // WindowManagerInterface* getWindowManager() const;
  // ros_integration::RosNodeAbstractionIface::WeakPtr getRosNodeAbstraction() const;
  // FrameManagerIface* getFrameManager() const;
  uint64_t getFrameCount() const;
  void notifyConfigChanged();
  // DisplayFactory* getDisplayFactory() const;
  // properties::PropertyTreeModel* getDisplayTreeModel() const;
  void emitStatusUpdate(const QString& message);
  // DisplayGroup* getRootDisplayGroup() const;
  uint32_t getDefaultVisibilityBit() const;
  // BitAllocator* visibilityBits();
  void setStatus(const QString& message);
  void setHelpPath(const QString& help_path);
  QString getHelpPath() const;
  rclcpp::Clock::SharedPtr getClock();

public Q_SLOTS:
  void resetTime();

Q_SIGNALS:
  void timeJumped();
  void preUpdate();
  void configChanged();
  void statusUpdate(const QString& message);
  void escapePressed();
};
};  // namespace rviz_common
