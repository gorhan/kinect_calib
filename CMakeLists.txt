cmake_minimum_required(VERSION 2.6)

set(KEYWORD "kinect_tf")

# Start a project.
project(${KEYWORD})

# Find YARP.  Point the YARP_DIR environment variable at your build.
find_package(YARP REQUIRED)
find_package(PCL 1.6 REQUIRED)
find_package(OpenCV REQUIRED)

# Search for source code.
file(GLOB folder_source src/*.cpp src/*.cc src/*.c)
file(GLOB folder_header include/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

# INCLUDE(iCubOptions)
# INCLUDE(iCubHelpers)

# Automatically add include directories if needed.
#foreach(header_file ${folder_header})
#  get_filename_component(p ${header_file} PATH)
#  include_directories(${p})
#endforeach(header_file ${folder_header})

# Inclue any directories needed for YARP
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Set up our main executable.
if (folder_source)
  add_executable(bin/${KEYWORD} ${folder_source} ${folder_header})
  target_link_libraries(bin/${KEYWORD} icubmod ${GSL_LIBRARIES} ${YARP_LIBRARIES} ${PCL_LIBRARIES})
else (folder_source)
  message(FATAL_ERROR "No source code files found. Please add something")
endif (folder_source)
