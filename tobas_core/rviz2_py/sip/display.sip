namespace rviz_common
{
// class Display
// {
class Display: rviz_common::Property /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz_common"
#include <rviz_common/display.hpp>
%End

public:
  Display();
  ~Display();

  // void initialize(DisplayContext* context);
  // QVariant getViewData(int column, int role) const;
  // Qt::ItemFlags getViewFlags(int column) const;
  QString getClassId() const;
  void setClassId(const QString& class_id);
  // void load(const Config& config);
  // void save(Config config) const;
  void setTopic(const QString& topic, const QString& datatype);
  bool isEnabled() const;
  void setFixedFrame(const QString& fixed_frame);
  void update(float wall_dt, float ros_dt);
  void reset();
  // void setStatus(properties::StatusProperty::Level level, const QString& name, const QString& text);
  // void setStatusStd(properties::StatusProperty::Level level, const std::string& name, const std::string& text);
  void setMissingTransformToFixedFrame(const std::string& frame, const std::string& additional_info = "");
  void setTransformOk();
  void deleteStatus(const QString& name);
  void deleteStatusStd(const std::string& name);
  void setVisibilityBits(uint32_t bits);
  void unsetVisibilityBits(uint32_t bits);
  uint32_t getVisibilityBits();
  // Ogre::SceneNode* getSceneNode() const;
  void setAssociatedWidget(QWidget* widget);
  QWidget* getAssociatedWidget() const;
  // PanelDockWidget* getAssociatedWidgetPanel();
  void setName(const QString& name) final;
  void emitTimeSignal(rclcpp::Time time);
  properties::Property* findProperty(const QString& name);
  bool updateFrame(const std::string& frame);
  bool updateFrame(const std::string& frame, rclcpp::Time time);

public Q_SLOTS:
  void setEnabled(bool enabled);
  void queueRender();
  void setIcon(const QIcon& icon);

Q_SIGNALS:
  void timeSignal(rviz_common::Display* display, rclcpp::Time time);
};
};  // namespace rviz_common
