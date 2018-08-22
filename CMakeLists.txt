SET(projectName bigDelivery)

# ---------------------------------------
# Declare a new CMake Project:
# ---------------------------------------
project( ${projectName} )

# These commands are needed by modern versions of CMake:
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
	if(POLICY CMP0043)
		cmake_policy(SET CMP0043 OLD) #  Ignore COMPILE_DEFINITIONS_<Config> properties.
	endif()
endif(COMMAND cmake_policy)

# ADD Subdirectory
ADD_SUBDIRECTORY(lib/libmodbus)
ADD_SUBDIRECTORY(lib/ganglia_driver_lib)
ADD_SUBDIRECTORY(lib/robotConnector)

INCLUDE_DIRECTORIES(${ganglia_driver_SOURCE_DIR})
INCLUDE_DIRECTORIES(${robotConnector_SOURCE_DIR})
INCLUDE_DIRECTORIES( ${wxWidgets_INCLUDE_DIRS} )

SET(EXECUTABLE_OUTPUT_PATH ".")

# force a 32-bit compile
if(NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(FATAL_ERROR "This program is incompatible with x64 build. Please switch to x86.")
endif()

set(wxWidgets_CONFIGURATION mswu)


# Include MRPT Library
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
FIND_PACKAGE(MRPT REQUIRED hwdrivers;gui;slam;nav)
FIND_PACKAGE(wxWidgets REQUIRED base;core;gl;adv;aui;html)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
SET(SRCS
bigDeliveryApp.cpp bigDeliveryApp.h
bigDeliveryMain.cpp bigDeliveryMain.h
MyGLCanvas.cpp MyGLCanvas.h
RobotNavInterface.cpp RobotNavInterface.h
BotModel.cpp BotModel.h
CPTG_TruckDrive_CollisionGridBased.cpp CPTG_TruckDrive_CollisionGridBased.h
CVehicleVelCmd_TruckDrive.cpp CVehicleVelCmd_TruckDrive.h
)

IF(WIN32)
	SET(SRCS ${SRCS} resource.rc)
ENDIF()

INCLUDE(${wxWidgets_USE_FILE})
ADD_DEFINITIONS(-DwxUSE_NO_MANIFEST=1)
LINK_DIRECTORIES(${wxWidgets_LIBRARY_DIRS})

# Define the executable target:
ADD_EXECUTABLE(${PROJECT_NAME} WIN32 ${SRCS})

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${wxWidgets_LIBRARIES} )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LIBS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ganglia_driver)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} robotConnector)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} legacy_stdio_definitions)