namespace rviz_common
{

class PanelDockWidget: QDockWidget /NoDefaultCtors/
{
%TypeHeaderCode
#include <rviz_common/panel_dock_widget.h>
%End

public:
  PanelDockWidget( const QString& name );

  void setContentWidget( QWidget* child );

signals:

  void visibilityChanged( bool );

};

};
