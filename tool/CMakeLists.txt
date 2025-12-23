
cmake_minimum_required(VERSION 3.5)

project(vanjee_driver_viewer)

message(=============================================================)
message("-- Ready to compile tools")
message(=============================================================)

include_directories(${DRIVER_INCLUDE_DIRS})

if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

if(WIN32)
  cmake_policy(SET CMP0074 NEW)
  set(OPENNI_ROOT "C:\\Program Files\\PCL 1.11.1\\3rdParty\\OpenNI2")
  set(OPENNI_LIBRARY "${OPENNI_ROOT}\\Lib\\OpenNI2.lib")
  set(OPENNI_INCLUDE_DIRS "${OPENNI_ROOT}\\Include\\")
  file(COPY ${OPENNI_ROOT}\\Redist\\OpenNI2.dll DESTINATION ${PROJECT_BINARY_DIR}\\Release)
  file(COPY ${OPENNI_ROOT}\\Redist\\OpenNI2.dll DESTINATION ${PROJECT_BINARY_DIR}\\Debug)
endif(WIN32)

find_package(PCL COMPONENTS common visualization io QUIET REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

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

if(WIN32)
    set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
    find_package(PCAP REQUIRED)
    include_directories(${PCAP_INCLUDE_DIR})
    list(APPEND EXTERNAL_LIBS ${PCAP_LIBRARY})
else()
    list(APPEND EXTERNAL_LIBS pcap)
endif(WIN32)

if(PCL_FOUND)

get_filename_component(LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
find_library(VANJEE_FILTER_LIB
    NAMES VanJeeLaser760Filter_linux
    PATHS ${LIB_DIR}
)

if((VANJEE_FILTER_LIB))
  message(STATUS "Found dynamic library")
  add_definitions(-DFILTER_DLL_ENABLE)
endif()

add_executable(vanjee_driver_viewer
               vanjee_driver_viewer.cpp)

target_link_libraries(vanjee_driver_viewer
                    ${EXTERNAL_LIBS}    
                    ${PCL_LIBRARIES}
                    ${VANJEE_DRIVER_LIBRARIES})

if((VANJEE_FILTER_LIB))
  target_link_libraries(vanjee_driver_viewer ${VANJEE_FILTER_LIB})
endif()

else()

message("PCL Not found! Can not compile vanjee_driver_viewer!")

endif()

install(TARGETS vanjee_driver_viewer
        RUNTIME DESTINATION /usr/bin)     
