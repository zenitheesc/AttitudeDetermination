cmake_minimum_required(VERSION 3.12)
project(libalglin VERSION 0.1.0)

# TARGET alglinear as header-only library


add_library(alglin INTERFACE)
target_compile_features(alglin INTERFACE cxx_std_11)
target_include_directories(alglin INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

target_compile_options(alglin INTERFACE -Wall -Wextra -Wshadow -pedantic)


# TESTING
Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v2.13.1)

FetchContent_MakeAvailable(Catch2)

add_executable(alglin-tests ${CMAKE_CURRENT_LIST_DIR}/tests/catch.cpp ${CMAKE_CURRENT_LIST_DIR}/tests/alglin-tests.cpp)
target_link_libraries(alglin-tests Catch2::Catch2)
target_link_libraries(alglin-tests alglin)


enable_testing()
add_test(NAME "AlgLin-Catch2" COMMAND alglin-tests)

