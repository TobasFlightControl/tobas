namespace rviz_common
{

class BoolProperty: rviz_common::Property /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz_common"
#include <rviz_common/properties/bool_property.h>
%End

public:
  BoolProperty( const QString& name = QString(),
                bool default_value = false,
                const QString& description = QString(),
                rviz_common::Property* parent = 0,
                const char *changed_slot = 0,
                QObject* receiver = 0 );

  virtual ~BoolProperty();

  virtual bool getBool() const;

public slots:
  bool setBool( bool value );
};

};
