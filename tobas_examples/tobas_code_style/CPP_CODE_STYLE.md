# C++ Code Style

## Example

See [tobas_cpp_code_style_example](./tobas_cpp_code_style_example/).

## C++ Version

- C++23
- Do not use non-standard extensions.

```cmake
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
```

## g++

- g++-13
- Compile options: `-Wall -Wextra -Wpedantic -Wshadow -Wswitch-enum -Werror`
- Enable PIC.

```cmake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Wswitch-enum -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
```

## File Extensions

- Use only `.hpp` and `.cpp`.

## Header Files

- Every `.cpp` file must have a same-named `.hpp` file, except in the following cases. In other words, do not collect the implementations of multiple header files into a single `.cpp` file.
  - Files that define a `main` function
  - Plugins, such as ROS 2 components or Gazebo plugins
- Use `#pragma once` for include guards.
- Directly include the header file that defines each symbol you use. Avoid depending on transitive includes whenever possible.
- Avoid forward declarations whenever possible.
- Separate function declarations from definitions.
- Mark definitions as inline when they are short and performance-sensitive.
- Include order:
  1. Related header file
  1. A blank line
  1. C system headers, and any other headers in angle brackets with the `.h` extension, e.g. `<unistd.h>`, `<stdlib.h>`.
  1. A blank line
  1. C++ standard library headers without file extensions, e.g. `<algorithm>`, `<cstddef>`.
  1. A blank line
  1. Third-party library headers
  1. A blank line
  1. Tobas library headers
  1. A blank line
  1. [ROS 2 common interface](https://github.com/ros2/common_interfaces/tree/jazzy) headers
  1. A blank line
  1. ROS 2 external interface headers
  1. A blank line
  1. ROS 2 Tobas interface headers, including type adapters
  1. A blank line
  1. Your project's headers
- Sort the headers in each section alphabetically.
- Do not use Unix directory aliases when including a related header: `../include/my_library/my_class.hpp` -> `my_library/my_class.hpp`
- Use double quotes only for headers from the same library.

<!-- Custom -->

- Prefer C++ standard library headers over C-style headers whenever possible.
  - e.g. `<string.h>` -> `<cstring>`, `<stdlib.h>` -> `<cstdlib>`

## Namespaces

- Put every symbol in the `tobas` namespace.
- Put symbols that may be included from another `.cpp` file in an appropriate namespace under `tobas`.
- `using namespace`: Do not use it in header files, and avoid it in source files whenever possible.
- If a symbol is defined in a `.cpp` file and used only with internal linkage, put it in an anonymous namespace to make it clear that other translation units cannot refer to it.
- Keep the scope of functions and variables as narrow as possible, and define variables as close as possible to where they are used.
- Global variables may be defined only as `constexpr`.

## Classes

- Do not put code that can fail in a constructor, because error handling is difficult there. Define an `initialize()` method instead.
- Do not call virtual methods from a base class constructor, because overridden implementations are not available while the base class constructor is running.
- Prefer marking constructors as `explicit` to prevent implicit type conversions.
- Define copy constructors and move constructors, and make it explicit whether each operation is allowed.
- Use `struct` when the type contains only `public` data. Otherwise, generally use `class`.
- Use `struct` instead of `std::pair` or `std::tuple` when doing so makes the meaning clearer.
- Use `public` inheritance.
- Operator overloads are allowed only when their meaning is clear.
- Member variables should generally be `private` with only the minimum necessary accessors. Other access levels are allowed if there is a valid reason and it is explained in a comment.
- Access groups must appear in this order:
  1. public
  1. protected
  1. private
- Within each access group, declarations must appear in this order:
  1. Types and type aliases, including `typedef`, `using`, `enum`, nested structs and classes, and friend types
  1. Non-static data members, optionally for structs only
  1. Static constants
  1. Factory functions
  1. Constructors and assignment operators
  1. Destructor
  1. All other functions, including static and non-static member functions and friend functions
  1. All other data members, including static and non-static data members
- Avoid `friend` whenever possible, and interact with other code only through `public` members.

<!-- Custom -->

- Use `= {}` for zero-initializing member variables.

## Functions

- For readability and performance, prefer return values over output arguments.
- Input arguments: pass primitive types by copy and non-primitive types by const reference.
- Output arguments: pass by non-const reference.
- Put input arguments before output arguments.
- Keep each function focused. One function should do one thing.
- Use overloads only when the meaning of the function does not change and all overloads can be described by the same doc comment.
- Do not give default values to virtual function arguments, because this can cause undefined behavior.
- ~~Define return types explicitly instead of using `auto`.~~ Use `auto` for return values unless there is a clear reason not to.

## Pointers

- Prefer references over pointers whenever possible.
- If you need a pointer, use a smart pointer instead of a raw pointer.
- Prefer `std::unique_ptr` over `std::shared_ptr` to make ownership clear.
- Do not use `std::auto_ptr`.
- Use `nullptr` instead of `NULL`.

<!-- Custom -->

- For null checks, write `if (!ptr)` instead of `if (ptr == nullptr)`.

## Exceptions

- Functions that may run on a mobile robot or vehicle must not throw exceptions.
- Use a `bool` return value or an error code for error handling.
- Specify `noexcept` after handling errors appropriately.

## Casts

- Avoid downcasts whenever possible.
- Use C++-style casts instead of C-style casts: `(int)x` -> `static_cast<int>(x)`

## Increment

- Use pre-increment unless there is a specific reason not to: `i++` -> `++i`

## const

- Add `constexpr` everywhere it can be used.
- Add `const` everywhere it can be used.

## Integer Types

- Do not use built-in integer types other than `int`.
- Prefer purpose-specific types such as `size_t`, `strdiff_t`, and `time_t` when appropriate.
- When a specific integer size is required, use the integer types defined in `stdint.h`.
- Do not use unsigned integer types except for special cases such as bit patterns. Use `assert` to guarantee non-negativity.

<!-- Custom -->

- Do not use non-standard types such as `uint`.

## Floating-Point Types

- Use only `float` or `double`.
- Do not use types such as `long double`.

## Naming Rules

- Names may be long, but they must make the meaning and purpose easy for readers to understand.

### Case

- Files: `hoge_fuga.hpp`, `hoge_fuga.cpp`
- Types, including classes, structs, type aliases, enums, and type template parameters: `HogeFuga`
- Local variables: `hoge_fuga`
- Global variables, discouraged: `g_hoge_fuga`
- Public variables: `hoge_fuga`
- Protected and private variables: `hoge_fuga_`
- Arguments: `_hoge_fuga`
- `constexpr` constants: `kHogeFuga`
- Functions: `hogeFuga()`
- Namespaces: `hoge_fuga`
- Enumerators, meaning enum elements: `kHogeFuga`
- Macros, discouraged: `HOGE_FUGA`

## Comments

- Doc comments, including class and function comments: `/** @brief ... */`
- Variable comments: `/* ... */` or `/** @brief ... */`
- Implementation comments: `// ...`
- Temporary commented-out code: `// ...`

<!-- Custom -->

- Write all comments in English.
- Add appropriate explanatory comments for each unit of processing.
- Prefer line breaks at natural phrase boundaries.

## License Comments

<!-- Custom -->

- Put the license text at the beginning of each file.

## Format

- Follow [.clang-format](../../.clang-format).

## ROS 2

<!-- Custom -->

- To support zero-copy delivery to unspecified subscribers, publish topics with `UniquePtr` and subscribe with `ConstSharedPtr`.
- Group parameter retrieval, Publisher creation, and Subscriber creation separately so the interface is easy to understand.

## Miscellaneous

<!-- Custom -->

- Use uppercase letters in hexadecimal numbers: `0x1a2b` -> `0x1A2B`
- Put a comma after the last enumerator in an enum.

## References

- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [Code style and language versions | ROS 2 Documentation](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html#id1)
