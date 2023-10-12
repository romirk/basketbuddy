cmake_minimum_required(VERSION 3.26)
project(december)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(december src/main.cpp
        src/rover.cpp
        include/rover.hpp
        include/ports.hpp
        src/ports.cpp
        src/sync.cpp
        include/sync.hpp)

target_link_libraries(december ${Boost_LIBRARIES})