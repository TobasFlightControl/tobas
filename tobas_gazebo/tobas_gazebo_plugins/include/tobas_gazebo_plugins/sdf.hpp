#pragma once

#include <sdf/sdf.hh>

namespace gazebo
{
bool getTurningDirection(const sdf::ElementConstPtr& sdf, int& dst);
}
