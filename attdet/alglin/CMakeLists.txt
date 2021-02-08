cmake_minimum_required(VERSION 3.8)
project(libalglin VERSION 0.1.0)

# TARGET alglinear as header-only library


add_library(alglin INTERFACE)
target_compile_options(alglin INTERFACE -std=gnu++11)
target_include_directories(alglin INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

set(ALGLIN_USE_FAST_INVSQRT OFF CACHE BOOL "Use FastInvSqrt Algorithm")
option(ALGLIN_USE_FAST_INVSQRT  "Use FastInvSqrt Algorithm") 

if(ALGLIN_USE_FAST_INVSQRT)
     target_compile_definitions(alglin INTERFACE USE_FAST_INVSQRT=1)
else()
     target_compile_definitions(alglin INTERFACE USE_FAST_INVSQRT=0)
endif()
target_compile_options(alglin INTERFACE
     $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:
          -Wall -Wextra -Wshadow -pedantic >
     $<$<CXX_COMPILER_ID:MSVC>:
          /W4>)

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

