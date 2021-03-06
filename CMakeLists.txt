cmake_minimum_required(VERSION 3.8)
project(AttitudeDetermination VERSION 0.1.0)

include(examples/serial/CMakeLists.txt)
include(examples/quest/CMakeLists.txt)
include(examples/websocket/CMakeLists.txt)


if(NOT TARGET all-tests)
  add_custom_target(all-tests)
  add_dependencies(all-tests alglin-tests attdet-tests)
endif()
enable_testing()
