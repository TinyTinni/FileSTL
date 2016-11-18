cmake_minimum_required (VERSION 3.5.1)
project (stl-Parser)

set (EXECUTABLE_NAME "stl-Parser-test")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Boost COMPONENTS unit_test_framework  REQUIRED)

set(SSU_Sources "stl_parser_test.cpp")

include_directories(${Boost_INCLUDE_DIR})
include_directories("../")

add_executable(${EXECUTABLE_NAME} ${SSU_Sources})

target_link_libraries(${EXECUTABLE_NAME} ${Boost_LIBRARIES})

add_definitions("-DSOURCE_DIR=\"${CMAKE_SOURCE_DIR}\"")

