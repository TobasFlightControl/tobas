#pragma once

namespace tobas
{
namespace my_namespace
{
class MyClass
{
public:
  int public_instance;

  inline int shortMethod();

  void longMethod();

private:
  int private_instance_;
};

inline int MyClass::shortMethod()
{
  return private_instance_;
}
}  // namespace my_namespace
}  // namespace tobas
