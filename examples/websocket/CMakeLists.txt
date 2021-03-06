cmake_minimum_required(VERSION 3.8)
project(websocket_demo VERSION 0.1.0)

if ( NOT TARGET attdet)
    include(${PROJECT_SOURCE_DIR}/attdet/CMakeLists.txt)
endif()

find_package(Poco REQUIRED Net Util)

add_executable(websocket ${CMAKE_CURRENT_LIST_DIR}/src/websocket.cpp)

target_link_libraries(websocket PUBLIC attdet)

target_link_libraries(websocket PUBLIC Poco::Net Poco::Util )

target_include_directories(websocket PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
