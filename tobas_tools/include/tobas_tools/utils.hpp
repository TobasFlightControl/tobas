#pragma once

#include <string>

#include <XYZgeomag.hpp>

namespace tobas
{
double getMass();

geomag::Elements geomag(double lat, double lon, double height);
}  // namespace tobas
