#pragma once

#include <iostream>

#include <eigen3/Eigen/Geometry>

std::ostream& operator<<(std::ostream& os, const Eigen::Quaterniond& arg);
