#pragma once

#include <string>

namespace tobas
{
namespace my_namespace
{
class MyClass
{
public:
  /* Types and type aliases */
  using Ptr = MyClass*;

  enum error_t
  {
    E_NO_ERROR,
    E_ERROR,
  };

  /* Static constants */
  static constexpr int kStaticConstant = 0;

  /* Factory functions */
  static MyClass FactoryFunction();

  /* Default constructor */
  explicit MyClass();

  /* Move constructor (permit) */
  MyClass(MyClass&& other) = default;
  MyClass& operator=(MyClass&& other) = default;

  /* Copy constructor (forbit) */
  MyClass(const MyClass& other) = delete;
  MyClass& operator=(const MyClass& other) = delete;

  /* Destructor */
  ~MyClass();

  /* All other functions */
  bool initialize();
  inline int shortMethod() const;
  error_t longMethod(int primitive_input, const std::string& non_primitive_input, double& output);

  /* All other data members */
  int public_instance;

protected:
  int protected_instance_;

private:
  int private_instance_;
};

inline int MyClass::shortMethod() const
{
  return private_instance_;
}
}  // namespace my_namespace
}  // namespace tobas
