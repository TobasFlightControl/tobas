#pragma once

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <srdfdom/model.h>

#include "./class_forward.hpp"
#include "./synchronized_string_parameter.hpp"

namespace tobas
{
TOBAS_CLASS_FORWARD(RDFLoader);  // Defines RDFLoaderPtr, ConstPtr, WeakPtr... etc

using NewModelCallback = std::function<void()>;

/** @class RDFLoader
 */
class RDFLoader
{
public:
  /**
   * @brief Default constructor
   *
   * Loads the URDF from a parameter given by the string argument,
   * and the SRDF that has the same name + the "_semantic" suffix
   *
   * If the parameter does not exist, attempt to subscribe to topics
   * with the same name and type std_msgs::msg::String.
   *
   * (specifying default_continuous_value/default_timeout allows users
   * to specify values without setting ros parameters)
   *
   * @param node ROS interface for parameters / topics
   * @param ros_name The string name corresponding to the URDF
   * @param default_continuous_value Default value for parameter with "_continuous" suffix.
   * @param default_timeout Default value for parameter with "_timeout" suffix.
   */
  RDFLoader(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& ros_name = "robot_description",
    bool default_continuous_value = false,
    double default_timeout = 10.);

  /* Initialize the robot model from a string representation of the URDF and SRDF documents */
  RDFLoader(const std::string& urdf_string, const std::string& srdf_string);

  /* Get the resolved parameter name for the robot description */
  const std::string& getRobotDescription() const
  {
    return ros_name_;
  }

  /* Get the URDF string*/
  const std::string& getURDFString() const
  {
    return urdf_string_;
  }

  /* Get the parsed URDF model*/
  const urdf::ModelInterfaceSharedPtr& getURDF() const
  {
    return urdf_;
  }

  /* Get the parsed SRDF model*/
  const srdf::ModelSharedPtr& getSRDF() const
  {
    return srdf_;
  }

  void setNewModelCallback(const NewModelCallback& cb)
  {
    new_model_cb_ = cb;
  }

  /* determine if given path points to a xacro file */
  static bool isXacroFile(const std::string& path);

  /* load file from given path into buffer */
  static bool loadFileToString(std::string& buffer, const std::string& path);

  /* run xacro with the given args on the file, return result in buffer */
  static bool
  loadXacroFileToString(std::string& buffer, const std::string& path, const std::vector<std::string>& xacro_args);

  /* helper that branches between loadFileToString() and loadXacroFileToString() based on result of isXacroFile() */
  static bool
  loadXmlFileToString(std::string& buffer, const std::string& path, const std::vector<std::string>& xacro_args);

  /* helper that generates a file path based on package name and relative file path to package */
  static bool loadPkgFileToString(
    std::string& buffer,
    const std::string& package_name,
    const std::string& relative_path,
    const std::vector<std::string>& xacro_args);

private:
  bool loadFromStrings();

  void urdfUpdateCallback(const std::string& new_urdf_string);
  void srdfUpdateCallback(const std::string& new_srdf_string);

  NewModelCallback new_model_cb_;

  std::string ros_name_;
  std::string urdf_string_, srdf_string_;

  SynchronizedStringParameter urdf_ssp_;
  SynchronizedStringParameter srdf_ssp_;

  srdf::ModelSharedPtr srdf_;
  urdf::ModelInterfaceSharedPtr urdf_;
};
}  // namespace tobas
