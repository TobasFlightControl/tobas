namespace rviz_common
{
// class DisplayGroup
// {
class DisplayGroup: rviz_common::Display /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz_common"
#include <rviz_common/display_group.hpp>
%End

public:
  DisplayGroup();
  ~DisplayGroup();

  Display* createDisplay(const QString& class_id);
  int numChildren() const;
  Property* childAtUnchecked(int index) const;
  Property* takeChildAt(int index);
  void addChild(Property* child, int index = -1);
  Qt::ItemFlags getViewFlags(int column) const;
  // void load(const Config& config);
  // void save(Config config) const;
  void addDisplay(Display* child);
  Display* takeDisplay(Display* child);
  void removeAllDisplays();
  int numDisplays() const;
  Display* getDisplayAt(int index) const;
  DisplayGroup* getGroupAt(int index) const;
  void update(float wall_dt, float ros_dt);
  void reset();

public Q_SLOTS:
  void onEnableChanged();

Q_SIGNALS:
  void displayAdded(Display* display);
  void displayRemoved(Display* display);
};
};  // namespace rviz_common
