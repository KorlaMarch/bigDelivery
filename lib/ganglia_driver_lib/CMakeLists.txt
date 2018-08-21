cmake_minimum_required(VERSION 2.8.3)
project(ganglia_driver)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${ganglia_driver_SOURCE_DIR})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

add_library(ganglia_driver
  ganglia_driver_lib/ganglia_multi_driver.h
  ganglia_multi_driver.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(hgr_ganglia ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(ganglia_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Declare a C++ executable
# add_executable(hgr_ganglia_node src/hgr_ganglia_node.cpp)


## Declare a C++ executable
# add_executable(obo_ganglia_driver_lib
#   src/obo_ganglia_driver_lib.cpp
#   src/hgr_modbus_ascii.cpp
# )
# add_executable(ganglia_test src/ganglia_test.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(hgr_ganglia_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(ganglia_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ganglia_driver libmodbus)

## Specify libraries to link a library or executable target against
# target_link_libraries(hgr_ganglia_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(ganglia_driver libmodbus)

# target_link_libraries(ganglia_test ganglia_driver)