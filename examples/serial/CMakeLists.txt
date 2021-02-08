cmake_minimum_required(VERSION 3.8)
project(serial_demo VERSION 0.1.0)


if ( NOT TARGET attdet)
    include(${PROJECT_SOURCE_DIR}/attdet/CMakeLists.txt)
endif()

add_executable(serial ${CMAKE_CURRENT_LIST_DIR}/src/serial.cpp)
target_link_libraries(serial attdet)
target_include_directories(serial PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
