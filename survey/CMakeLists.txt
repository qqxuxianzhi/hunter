# Set the minimum required version of cmake for a project and update Policy Settings
# to match the version given.
cmake_minimum_required(VERSION 3.1.0)

message(STATUS "CMAKE_ROOT = " ${CMAKE_ROOT})

# Sets project details such as name, version, etc. and enables languages.
# Stores the project name in the PROJECT_NAME variable. Additionally this sets variables:
# PROJECT_SOURCE_DIR, <PROJECT-NAME>_SOURCE_DIR
# PROJECT_BINARY_DIR, <PROJECT-NAME>_BINARY_DIR
project(survey)
message(STATUS "PROJECT_NAME = " ${PROJECT_NAME})
message(STATUS "PROJECT_SOURCE_DIR = " ${PROJECT_SOURCE_DIR})
message(STATUS "PROJECT_BINARY_DIR = " ${PROJECT_BINARY_DIR})

# Control if...else... statement format
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS ON)

# Automatically add the current source and build directories to the include path.
# If this variable is enabled, CMake automatically add CMAKE_CURRENT_SOURCE_DIR and
# CMAKE_CURRENT_BINARY_DIR to the include path for each directory. These additional
# include directories do not propagate down to subdirectories. This is useful mainly
# for out-of-source builds, where files generated into the build tree are included by
# files located in the source tree.
set(CMAKE_INCLUDE_CURRENT_DIR ON)
message(STATUS "CMAKE_CURRENT_SOURCE_DIR = " ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "CMAKE_CURRENT_BINARY_DIR = " ${CMAKE_CURRENT_BINARY_DIR})

# Default compilation flags to be used when compiling CXX(C++) files. Will only be used
# by CMake on the first configuration to determine CXX default compilation flags, after
# which the value for CXXFLAGS is stored in the cache as CMAKE_CXX_FLAGS. For any
# configuration run (including the first), the environment variable will be ignored if
# the CMAKE_CXX_FLAGS variable is defined.
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC")
#set(CMAKE_CXX_FLAGS "-fPIC")


# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
# set(CMAKE_AUTOUIC ON)
# Handle the Qt rcc code generator automatically
# set(CMAKE_AUTORCC ON)

# Enable GTest
set(ENABLE_GTEST 1)
if(ENABLE_GTEST)
  add_definitions(-DENABLE_GTEST=1)
endif()

# Enable HMI
set(ENABLE_HMI 0)
if(ENABLE_HMI)
  add_definitions(-DENABLE_HMI=1)
endif()

# can driver options:  1 - kvaser, 2 - emuc2
set(ENABLE_CAN_DEV_KVASER 0)
set(ENABLE_CAN_DEV_EMUC2 0)
set(ENABLE_CAN_DEV_ZLGCANNET 1)
set(ENABLE_CAN_DEV_MDC 0)
set(ENABLE_SERIAL_DEV_POSIX 1)
set(ENABLE_SERIAL_DEV_MDC 0)
if(ENABLE_CAN_DEV_KVASER)
  add_definitions(-DENABLE_CAN_DEV_KVASER=1)
endif()
if(ENABLE_CAN_DEV_EMUC2)
  add_definitions(-DENABLE_CAN_DEV_EMUC2=1)
endif()
if(ENABLE_CAN_DEV_ZLGCANNET)
  add_definitions(-DENABLE_CAN_DEV_ZLGCANNET=1)
endif()
if(ENABLE_CAN_DEV_MDC)
  add_definitions(-DENABLE_CAN_DEV_MDC=1)
endif()
if(ENABLE_SERIAL_DEV_POSIX)
  add_definitions(-DENABLE_SERIAL_DEV_POSIX=1)
endif()
if(ENABLE_SERIAL_DEV_MDC)
  add_definitions(-DENABLE_SERIAL_DEV_MDC=1)
endif()

# Find the QtWidgets library
# find_package(Qt5Widgets COMPONENTS Qt5Core Qt5Gui Qt5OpenGL OpenGL)
find_package(Qt5Widgets CONFIG REQUIRED)
message(STATUS "Qt5Widgets_FOUND = " ${Qt5Widgets_FOUND})
message(STATUS "Qt5Widgets_VERSION = " ${Qt5Widgets_VERSION})
message(STATUS "Qt5Widgets_LIBRARIES = " ${Qt5Widgets_LIBRARIES})
message(STATUS "Qt5Widgets_INCLUDE_DIRS = " ${Qt5Widgets_INCLUDE_DIRS})

# Enable OpenGL
set(QT_USE_QTOPENGL TRUE)
find_package(Qt5OpenGL)
message(STATUS "Qt5OpenGL_FOUND = " ${Qt5OpenGL_FOUND})
message(STATUS "Qt5OpenGL_VERSION = " ${Qt5OpenGL_VERSION})
message(STATUS "Qt5OpenGL_LIBRARIES = " ${Qt5OpenGL_LIBRARIES})
message(STATUS "Qt5OpenGL_INCLUDE_DIRS = " ${Qt5OpenGL_INCLUDE_DIRS})

# Using ROS
# Find catkin macros and libraries
# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp message_generation)
message(STATUS "catkin_FOUND = " ${catkin_FOUND})
message(STATUS "catkin_VERSION = " ${catkin_VERSION})
message(STATUS "catkin_LIBRARIES = " ${catkin_LIBRARIES})
message(STATUS "catkin_INCLUDE_DIRS = " ${catkin_INCLUDE_DIRS})
message(STATUS "roscpp_FOUND = " ${roscpp_FOUND})
message(STATUS "roscpp_VERSION = " ${roscpp_VERSION})
message(STATUS "roscpp_LIBRARIES = " ${roscpp_LIBRARIES})
message(STATUS "roscpp_INCLUDE_DIRS = " ${roscpp_INCLUDE_DIRS})
###################################
## catkin specific configuration ##
###################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if your package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES agitr
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
# )
#catkin_package(
#)


add_service_files( 
  DIRECTORY src/outside_msg/srv
  FILES
  SurveyServer.srv
  DrawLine.srv
  WriteWord.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
# CMAKE_RUNTIME_OUTPUT_DIRECTORY
# Output directory in which to build RUNTIME target files.
# This property specifies the directory into which runtime target files should be built.
# Multi configuration generators (VS, Xcode) append a per-configuration subdirectory to
# the specified direcotory.
# There are three kinds of target files that may be built: archive, library, and runtime.
# Executables are always treated as runtime targets. Static libraries are always treated
# as archive targets. Module libraries are always treated as library targets. For non-DLL
# platforms shared libraries are treated as library targets. For DLL platforms the DLL part
# of a shared library is treated as a runtime target and the corresponding import library
# is treated as an archive target. All Windows based systems including Cygwin are DLL platforms.
# This property is initialized by the value of the variable CMAKE_RUNTIME_OUTPUT_DIRECTORY
# if it is set when a target is created.
message(STATUS "After load catkin:")
message(STATUS "CMAKE_RUNTIME_OUTPUT_DIRECTORY = " ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# Add the given directories to those the compiler uses to search for include files.
# Relative paths are interpreted as relative to the current source directory.
include_directories(
  "src"
    "src/common/include"
    "src/common/src"
    "src/ad_msg/include"
    "src/data_serial/include"
    "src/veh_model/include"
    "src/veh_model/src"
    "src/outside_msg/msg_series_c/include"
    "src/outside_msg/msg_series_c/include/map_apollo"
    "src/outside_msg/msg_series_d600/include"
    "src/dev_driver"
    "src/framework"
    "src/hmi"
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    "~/3rdparty/lcm-1.4.0/include/"
    "~/3rdparty/protobuf-3.5.1/include/" 
    "~/3rdparty/gflags-2.2.1/include/"
    "~/3rdparty/glog-0.3.3/include/"
    "~/3rdparty/gtest-1.7.0/include/"
    "~/3rdparty/zmotion/include/"
    ${SVGPP_INCLUDE_DIR}
)
if(ENABLE_CAN_DEV_KVASER)
  # include kvaser
endif()
if(ENABLE_CAN_DEV_EMUC2)
  # include emuc2
  include_directories("~/3rdparty/emuc2/include")
endif()

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
message(STATUS "Show include directories:---------")
foreach(dir ${dirs})
    message(STATUS ${dir})
endforeach()
message(STATUS "----------------------------------")

# Specify the paths in which the linker should search for libraries. The command will
# apply only to targets created after it is called. Relative paths given to this command
# are interpreted as relative to the current source direcotry.
# Note that this command is rarely necessary. Library locations returned by find_package()
# and find_library() are absolute paths. Pass these absolute library file path directly to
# the target_link_libraries() command. CMake will ensure the linker finds them.
link_directories(
  "/usr/local/lib"
    "~/3rdparty/protobuf-3.5.1/lib/"
    "~/3rdparty/lcm-1.4.0/lib/"
    "~/3rdparty/gflags-2.2.1/lib/"
    "~/3rdparty/glog-0.3.3/lib/"
    "~/3rdparty/gtest-1.7.0/lib/"
    "~/3rdparty/zmotion/lib/"
)
if(ENABLE_CAN_DEV_KVASER)
  # include kvaser
endif()
if(ENABLE_CAN_DEV_EMUC2)
  # include emuc2
  link_directories("~/3rdparty/emuc2")
endif()

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY LINK_DIRECTORIES)
message(STATUS "Show link directories:")
foreach(dir ${dirs})
    message(STATUS ${dir})
endforeach()
message(STATUS "----------------------------------")

# Add ui and qrc files
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/hmi/ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/hmi/resources/*.qrc)
message(STATUS "QT_FORMS = " ${QT_FORMS})
message(STATUS "QT_RESOURCES = " ${QT_RESOURCES})
#
if (Qt5Widgets_FOUND)
#   Create code from a list of Qt resource files
    qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
#   Create code from a list of Qt designer ui files
    qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
else ()
    message(FATAL_ERROR "Qt5Widgets not found")
endif ()

# Add source files
file(GLOB_RECURSE SRC_LIST RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
    src/*.hpp
    src/*.h
    src/*.c
    src/*.cc
    src/*.cpp
)
message(STATUS "SRC_LIST = " ${SRC_LIST})

# Add an executable to the project using the specified source files.
add_executable(${PROJECT_NAME}
    ${SRC_LIST}
    ${QT_RESOURCES_CPP} ${QT_FORMS_HPP}
)

# Specify libraries or flags to use when linking a given target and/or its dependents.
target_link_libraries(${PROJECT_NAME}
    Qt5Widgets Qt5Core Qt5Gui Qt5OpenGL
    GL GLU glut
    ${catkin_LIBRARIES}
    protobuf glog pthread
    boost_thread boost_date_time boost_system boost_chrono
    lcm
    zmotion
)
if(ENABLE_GTEST)
  target_link_libraries(${PROJECT_NAME} gtest gtest_main)
endif()
if(ENABLE_CAN_DEV_KVASER)
  # link kvaser
  target_link_libraries(${PROJECT_NAME} canlib)
endif()
if(ENABLE_CAN_DEV_EMUC2)
  # link emuc2
  target_link_libraries(${PROJECT_NAME} lib_emuc2_64.a)
endif()

