cmake_minimum_required(VERSION 3.5)

project(vanjee_driver_demos)

message(=============================================================)
message("-- Ready to compile demos")
message(=============================================================)

if (${ENABLE_PCL_POINTCLOUD})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

endif (${ENABLE_PCL_POINTCLOUD})

option(DISABLE_PCAP_PARSE         "Disable PCAP file parse" ON) 
if(${DISABLE_PCAP_PARSE})
  add_definitions("-DDISABLE_PCAP_PARSE")
else()
  if(WIN32)
    set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
    find_package(PCAP REQUIRED)
    include_directories(${PCAP_INCLUDE_DIR})
    list(APPEND EXTERNAL_LIBS ${PCAP_LIBRARY})
  else()
    list(APPEND EXTERNAL_LIBS pcap)
  endif(WIN32)
endif(${DISABLE_PCAP_PARSE})

option(ENABLE_TRANSFORM "Enable transform functions" OFF)
if(${ENABLE_TRANSFORM})
 add_definitions("-DENABLE_TRANSFORM")

 find_package(Eigen3 REQUIRED)
 include_directories(${EIGEN3_INCLUDE_DIR})
endif(${ENABLE_TRANSFORM})

option(ENABLE_GTEST "Enable gtest value" OFF)
if(${ENABLE_GTEST})
 add_definitions("-DENABLE_GTEST")
endif(${ENABLE_GTEST})

option(ENABLE_TIMESTAMP_TRANS_WITH_LOCAL_TIME_ZONE "Enable transform timestamp to datetime with local time zone" OFF)
if(${ENABLE_TIMESTAMP_TRANS_WITH_LOCAL_TIME_ZONE})
  add_definitions("-DENABLE_TIMESTAMP_TRANS_WITH_LOCAL_TIME_ZONE")
endif(${ENABLE_TIMESTAMP_TRANS_WITH_LOCAL_TIME_ZONE})

option(ENABLE_GRAVITY_ACCELERATION_REMOVE      "Enable Gravity acceleration remove" OFF)
if(${ENABLE_GRAVITY_ACCELERATION_REMOVE})
  add_definitions("-DENABLE_GRAVITY_ACCELERATION_REMOVE")
endif(${ENABLE_GRAVITY_ACCELERATION_REMOVE})

find_package(vanjee_driver REQUIRED)
include_directories(${VANJEE_DRIVER_INCLUDE_DIRS})

get_filename_component(LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
find_library(VANJEE_FILTER_LIB
    NAMES VanJeeLaser760Filter_linux
    PATHS ${LIB_DIR}
)

if((VANJEE_FILTER_LIB))
  message(STATUS "Found dynamic library")
  add_definitions(-DFILTER_DLL_ENABLE)
endif()

add_executable(demo_online
              demo_online.cpp)
target_link_libraries(demo_online
                    ${EXTERNAL_LIBS} ${VANJEE_DRIVER_LIBRARIES})

if((VANJEE_FILTER_LIB))
  target_link_libraries(demo_online ${VANJEE_FILTER_LIB})
endif()

add_executable(demo_online_multi_lidars
              demo_online_multi_lidars.cpp)

target_link_libraries(demo_online_multi_lidars
                    ${EXTERNAL_LIBS} ${VANJEE_DRIVER_LIBRARIES})

if((VANJEE_FILTER_LIB))
  target_link_libraries(demo_online_multi_lidars ${VANJEE_FILTER_LIB})
endif()

if(NOT ${DISABLE_PCAP_PARSE})

add_executable(demo_pcap
               demo_pcap.cpp)

target_link_libraries(demo_pcap
                     ${EXTERNAL_LIBS} ${VANJEE_DRIVER_LIBRARIES})

if((VANJEE_FILTER_LIB))
  target_link_libraries(demo_pcap ${VANJEE_FILTER_LIB})
endif()

else()

add_definitions("-DDISABLE_PCAP_PARSE")
                     
endif(NOT ${DISABLE_PCAP_PARSE})


        
