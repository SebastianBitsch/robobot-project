cmake_minimum_required(VERSION 2.6)
project(socket_server_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -o0 -Wall -pedantic \
    -Wno-format-truncation -Wno-return-type \
    -std=c++11 ${EXTRA_CC_FLAGS}")
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-pthread")

add_executable(socket_server_test main.cpp ulock.cpp ulogfile.cpp umsgqueue.cpp userverclient.cpp userverport.cpp userverqueue.cpp utime.cpp)

target_link_libraries(socket_server_test ${CMAKE_THREAD_LIBS_INIT} -lpthread)

install(TARGETS socket_server_test RUNTIME DESTINATION bin)
