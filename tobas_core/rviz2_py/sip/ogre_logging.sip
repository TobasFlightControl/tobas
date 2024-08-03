namespace rviz_common
{

class OgreLogging /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz_common"
#include <rviz_common/ogre_helpers/ogre_logging.h>
%End

public:
  /** @brief Configure Ogre to write output using the ROS logger. */
  static void useRosLog();

  /** @brief Configure Ogre to write output to the given log file
   * name.  If file name is a relative path, it will be relative to
   * the directory which is current when the program is run.  Default
   * is "Ogre.log". */
  static void useLogFile( const QString& filename = "Ogre.log" );

  /** @brief Disable Ogre logging entirely.  This is the default. */
  static void noLog();
};

};
