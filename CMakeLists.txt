cmake_minimum_required (VERSION 3.5.1)
project (stl-Parser)

include(CTest)
if(BUILD_TESTING)
    add_subdirectory("test")
endif()
