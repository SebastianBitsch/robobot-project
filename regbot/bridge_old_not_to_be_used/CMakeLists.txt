cmake_minimum_required(VERSION 2.6)
project(robobot_bridge)
find_package(OpenCV REQUIRED )
find_package(Threads REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
execute_process(COMMAND uname -m RESULT_VARIABLE IS_OK OUTPUT_VARIABLE CPU1)
string(STRIP ${CPU1} CPU)
if (${CPU} MATCHES "armv7l")
   message("got RASPBERRY ${CPU}")
   #    set(EXTRA_CC_FLAGS " -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s -DRASPBERRY_PI -D${CPU}")
   set(EXTRA_CC_FLAGS "-D${CPU} -O2 -g0 -DRASPBERRY_PI")
   set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2 -DRASPBERRY_PI")
   set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2 -DRASPBERRY_PI")
else()
   message("not RASPBERRY ${CPU}")
   set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic \
    -Wno-format-truncation -Wno-return-type \
    -std=c++11 ${EXTRA_CC_FLAGS}")
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-pthread")
add_definitions(-Draspicam_CV_LIBS)
add_executable(robobot_bridge main.cpp 
      utime.cpp ujoy.cpp userverport.cpp userverclient.cpp 
      urun.cpp uregbot.cpp udata.cpp uhandler.cpp udataitem.cpp 
      Adafruit_GFX.cpp ArduiPi_OLED.cpp bcm2835.c Wrapper.cpp uoled.cpp
      ulogfile.cpp tcpCase.cpp ubridge.cpp)
target_link_libraries(robobot_bridge ${CMAKE_THREAD_LIBS_INIT})
install(TARGETS robobot_bridge RUNTIME DESTINATION bin)
