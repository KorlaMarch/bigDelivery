cmake_minimum_required(VERSION 2.8.3)
project(robotConnector)

## Specify additional locations of header files
include_directories(${robotConnector_SOURCE_DIR})
include_directories(${ganglia_driver_SOURCE_DIR})

SET(SRCS
RobotConnector.cpp RobotConnector.h
SerialPort.cpp SerialPort.h
TruckData.h
)

add_library(robotConnector ${SRCS})

FIND_PACKAGE(MRPT REQUIRED hwdrivers;base)

## Add cmake target dependencies of the executable
add_dependencies(robotConnector ganglia_driver)

## Specify libraries to link a library or executable target against
target_link_libraries(robotConnector ganglia_driver)
target_link_libraries(robotConnector ${MRPT_LIBS})

add_executable(robotConnectorTest RobotConnectorTest.cpp)
target_link_libraries(robotConnectorTest ${MRPT_LIBS})
target_link_libraries(robotConnectorTest robotConnector)