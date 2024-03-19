#include "../include/tobas_tools/command_level_handler.hpp"

namespace tobas
{
CommandLevelHandler::CommandLevelHandler()
{
}

bool CommandLevelHandler::update(const uint8_t& new_level, const ros::Time& cur_time)
{
  if (new_level >= cur_level_ || (cur_time - t_last_highest_level_).toSec() > kHighestLevelTimeout)
  {
    cur_level_ = new_level;
    t_last_highest_level_ = cur_time;
    return true;
  }
  else
  {
    return false;
  }
}
}  // namespace tobas
