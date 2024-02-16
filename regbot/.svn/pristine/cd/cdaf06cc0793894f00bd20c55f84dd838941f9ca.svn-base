cmake_minimum_required(VERSION 2.6)
project(robobot)

find_package(Threads REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -std=c++17 ${EXTRA_CC_FLAGS} -Wno-psabi")
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-pthread")

add_executable(mission main.cpp bridge.cpp mission.cpp)

target_link_libraries(mission ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS mission RUNTIME DESTINATION bin)
