cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE} -std=Release")

project(kylin)

find_package(OpenCV REQUIRED)

if(NOT WIN32)
ADD_DEFINITIONS("-std=c++0x -O3")
endif(NOT WIN32)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
include_directories(asp/inc) 
include_directories(uart/linux) 
FILE(GLOB sourcefiles "asp/src/*.c" "uart/linux/*.cpp" "*.cpp" "*.c")
add_executable( kylin ${sourcefiles} )

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
target_link_libraries( kylin pthread ${OpenCV_LIBS})
 



