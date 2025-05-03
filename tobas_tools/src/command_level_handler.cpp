#include "../include/tobas_tools/command_level_handler.hpp"

using namespace std;

namespace tobas
{
CommandLevelHandler::CommandLevelHandler()
{
}

bool CommandLevelHandler::update(const uint8_t& new_level, const rclcpp::Time& cur_time)
{
  if (new_level >= cur_level_ || (cur_time - t_last_highest_level_).seconds() > kHighestLevelTimeout) {
    if (new_level != cur_level_) {
      cout << (int)cur_level_ << " to " << (int)new_level << "." << endl;
    }

    cur_level_ = new_level;
    t_last_highest_level_ = cur_time;
    return true;
  }
  else {
    return false;
  }
}
}  // namespace tobas
