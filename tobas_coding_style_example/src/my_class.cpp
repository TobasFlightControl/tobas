#include "tobas_coding_style_example/my_class.hpp"

#include <stdlib.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>

#include <eigen3/Eigen/Core>

#include <tobas_math/core.hpp>

#include "tobas_coding_style_example/util.hpp"

namespace tobas
{
namespace my_namespace
{
namespace
{
void internalMethod()
{
}
}  // namespace

void MyClass::longMethod()
{
  internalMethod();
}
}  // namespace my_namespace
}  // namespace tobas
