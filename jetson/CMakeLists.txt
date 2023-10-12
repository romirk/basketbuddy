cmake_minimum_required(VERSION 3.22)
project(december)

set(CMAKE_CXX_STANDARD 17)
find_package(Threads)
find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})

file(GLOB SOURCES CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB HEADERS CONFIGURE_DEPENDS "include/*.hpp")

include_directories(include)
add_executable(december ${SOURCES} ${HEADERS})
target_include_directories(december PRIVATE include)
target_link_libraries(december ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
