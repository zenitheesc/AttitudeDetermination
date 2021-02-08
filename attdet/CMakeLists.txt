cmake_minimum_required(VERSION 3.8)
project(libattdet VERSION 0.1.0)

include(${CMAKE_CURRENT_LIST_DIR}/alglin/CMakeLists.txt)

add_library(attdet  ${CMAKE_CURRENT_LIST_DIR}/src/attdet.cpp)
target_include_directories(attdet PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(attdet alglin)

# TESTING
Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v2.13.1)

FetchContent_MakeAvailable(Catch2)

add_executable(attdet-tests ${CMAKE_CURRENT_LIST_DIR}/tests/catch.cpp ${CMAKE_CURRENT_LIST_DIR}/tests/attdet-tests.cpp)
target_link_libraries(attdet-tests Catch2::Catch2)
target_link_libraries(attdet-tests attdet)


enable_testing()
add_test(NAME "AttDet-Catch2" COMMAND attdet-tests)

