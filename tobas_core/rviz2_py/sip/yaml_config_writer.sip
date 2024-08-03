namespace rviz_common
{

class YamlConfigWriter /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz_common"
#include <rviz_common/yaml_config_writer.h>
%End

public:
  /** @brief Constructor.  Writer starts in a non-error state with no status message. */
  YamlConfigWriter();

  /** @brief Write config data to a file.  This potentially changes
   * the return values of error() and statusMessage(). */
  void writeFile( const rviz_common::Config& config, const QString& filename );

  /** @brief Write config data to a string, and return it.  This
   * potentially changes the return values of error() and
   * statusMessage(). */
  QString writeString( const rviz_common::Config& config, const QString& filename = "data string" );

  /** @brief Write config data to a std::ostream.  This potentially
   * changes the return values of error() and statusMessage(). */
//  void writeStream( const rviz_common::Config& config, std::ostream& out, const QString& filename = "data stream" );

  /** @brief Return true if the latest write operation had an error. */
  bool error();

  /** @brief Return an error message if the latest write call had an
   * error, or the empty string if there was no error. */
  QString errorMessage();
};

};
