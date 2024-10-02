#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

#include <tobas_linux/process_settings.hpp>

bool init(int argc, char* argv[])
{
  linux::ProcessSettings settings;
  if (!settings.init(argc, argv))
    return false;

  try
  {
    settings.configureProcess();
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }

  rclcpp::init(argc, argv);

  return true;
}
