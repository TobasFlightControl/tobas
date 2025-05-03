#include <iostream>

#include <tobas_code_style_example/my_class.hpp>

int main()
{
  tobas::my_namespace::MyClass my_instance;

  if (!my_instance.initialize())
  {
    std::cerr << "Failed to initialize MyClass." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
