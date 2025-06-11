
# - Config file for the  package
# It defines the following variables
#  vanjee_driver_INCLUDE_DIRS - include directories for 
#  vanjee_driver_LIBRARIES    - libraries to link against
#  vanjee_driver_FOUND        - found flag

if(WIN32)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit
    set(Boost_ARCHITECTURE "-x64")
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 4) # 32-bit
    set(Boost_ARCHITECTURE "-x32")
  endif()
  set(Boost_USE_STATIC_LIBS ON)
  set(Boost_USE_MULTITHREADED ON)
  set(Boost_USE_STATIC_RUNTIME OFF)
endif(WIN32)

if(${ENABLE_TRANSFORM})
  add_definitions("-DENABLE_TRANSFORM")
endif(${ENABLE_TRANSFORM})

set(vanjee_driver_INCLUDE_DIRS "/home/vanjee/ROS/01vanjee_sdk_dev/00temp_workspace/src/vanjee_lidar_sdk/src/vanjee_driver/src;/usr/local/Project/include")
set(VANJEE_DRIVER_INCLUDE_DIRS "/home/vanjee/ROS/01vanjee_sdk_dev/00temp_workspace/src/vanjee_lidar_sdk/src/vanjee_driver/src;/usr/local/Project/include")

set(vanjee_driver_LIBRARIES "pthread")
set(VANJEE_DRIVER_LIBRARIES "pthread")

set(vanjee_driver_FOUND true)
set(VANJEE_DRIVER_FOUND true)
