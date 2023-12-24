#pragma once

#include <iostream>
#include <Eigen/Geometry>

std::ostream& operator<<(std::ostream& os, const Eigen::Quaterniond& arg);
