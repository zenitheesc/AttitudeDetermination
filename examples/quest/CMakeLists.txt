cmake_minimum_required(VERSION 3.12)
project(quest_demo VERSION 0.1.0)

if ( NOT TARGET attdet)
    include(${PROJECT_SOURCE_DIR}/attdet/CMakeLists.txt)
endif()
add_executable(quest ${CMAKE_CURRENT_LIST_DIR}/src/quest.cpp)
target_link_libraries(quest attdet)
target_include_directories(quest PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)

add_custom_command(
        TARGET quest
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo `${PROJECT_SOURCE_DIR}/scripts/text-size.sh`
        COMMENT "Print binary text section size"
)
