include_guard(GLOBAL)

# Default single-configuration builds to Release, preserving an explicitly selected configuration.
function(tobas_default_build_type)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
  endif()
endfunction()

# Apply project requirements only to this compiled target, without exporting warnings to its consumers. NO_SWITCH_ENUM
# retains the existing exception for component_manager in tobas_components_rt.
function(tobas_target_defaults target)
  cmake_parse_arguments(PARSE_ARGV 1 arg "NO_SWITCH_ENUM" "" "")
  if(arg_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "Unknown tobas_target_defaults arguments: ${arg_UNPARSED_ARGUMENTS}")
  endif()
  set_target_properties(
    ${target}
    PROPERTIES CXX_STANDARD 23
               CXX_STANDARD_REQUIRED ON
               CXX_EXTENSIONS OFF
               POSITION_INDEPENDENT_CODE ON
  )
  target_compile_options(${target} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic;-Wshadow;-Werror>")
  if(NOT arg_NO_SWITCH_ENUM)
    target_compile_options(${target} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:-Wswitch-enum>")
  endif()
endfunction()
