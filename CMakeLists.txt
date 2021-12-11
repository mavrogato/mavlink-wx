
cmake_minimum_required(VERSION 3.11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_COMPILER /usr/bin/g++-10)
set(CMAKE_CXX_FLAGS "-Wall -Werror")

project(mavlink-wx)

find_package(wxWidgets REQUIRED gl core base propgrid)
include(${wxWidgets_USE_FILE})

include(FetchContent)
FetchContent_Populate(mavlink
  GIT_REPOSITORY https://github.com/mavlink/c_library_v2)

include_directories(
  ${mavlink_SOURCE_DIR})

add_executable(mavlink-wx main.cc)

target_compile_options(mavlink-wx
  PRIVATE
  -std=c++20
  -fcoroutines)

target_link_libraries(mavlink-wx
  PRIVATE
  ${wxWidgets_LIBRARIES})

add_custom_target(run
  DEPENDS mavlink-wx
  COMMAND mavlink-wx)
