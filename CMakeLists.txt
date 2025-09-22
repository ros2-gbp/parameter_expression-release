cmake_minimum_required(VERSION 3.5)
project(parameter_expression)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wfloat-equal)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# muParser
find_package(muparser QUIET)

if(muparser_FOUND)
  message(STATUS "Using system MuParser")

  # Add uppercase alias if only the lowercase target is defined
  if(NOT TARGET MuParser::MuParser)
    add_library(MuParser::MuParser ALIAS muparser::muparser)
  endif()
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(MuParser GLOBAL IMPORTED_TARGET muparser)

  if(MuParser_FOUND)
    message(STATUS "Using system MuParser (via pkg-config)")
    add_library(MuParser::MuParser ALIAS PkgConfig::MuParser)
  endif()
endif()

include_directories(SYSTEM ${muparser_INCLUDE_DIRS})

# Lib
file(
  GLOB_RECURSE sources
  "./src/**/*.cpp"
  "./src/**/*.c"
  "./src/*.cpp"
  "./src/*.c"
)

ament_auto_add_library(${PROJECT_NAME} SHARED ${sources})

target_link_libraries(${PROJECT_NAME} MuParser::MuParser)

ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  file(
    GLOB_RECURSE test_sources
    "./test/*.cpp"
  )
  ament_add_gmock(${PROJECT_NAME}_test ${test_sources})

  target_include_directories(${PROJECT_NAME}_test PUBLIC include)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_auto_package()