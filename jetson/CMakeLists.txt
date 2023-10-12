cmake_minimum_required(VERSION 3.22)
project(december)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})

set(SOURCES
        src/main.cpp
        src/rover.cpp
        src/ports.cpp
        src/sync.cpp
        src/config.cpp
)

set(HEADERS
        include/rover.hpp
        include/ports.hpp
        include/sync.hpp
        include/config.hpp
)

include_directories(include)
add_executable(december ${SOURCES} ${HEADERS})
target_include_directories(december PRIVATE include)
target_link_libraries(december ${Boost_LIBRARIES})