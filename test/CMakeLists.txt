cmake_minimum_required (VERSION 3.5.1)
project (stl-Parser)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package (Boost REQUIRED COMPONENTS unit_test_framework)

set(SSU_Sources "stl_parser_test.cpp")
add_executable(std_parser_test ${SSU_Sources})

target_link_libraries(std_parser_test PRIVATE Boost::unit_test_framework)

add_test(NAME std_parser_test COMMAND std_parser_test)

target_add_definitions(std_parser_test PRIVATE "-DSOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

