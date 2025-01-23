cmake_minimum_required(VERSION 3.0.2)
project(awv_fusion)

add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
  eigen_conversions
)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(fmt PATHS "/usr/local/include/fmt" REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES awv_fusion
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs tf eigen_conversions
)

# Manual GeographicLib configuration
set(GeographicLib_INCLUDE_DIRS "/usr/include" CACHE PATH "Path to GeographicLib include directory")
set(GeographicLib_LIBRARIES "/usr/lib/x86_64-linux-gnu/libGeographic.so" CACHE FILEPATH "Path to GeographicLib library")

# Verify the paths
if(NOT EXISTS ${GeographicLib_INCLUDE_DIRS}/GeographicLib/Config.h)
    message(FATAL_ERROR "GeographicLib headers not found at ${GeographicLib_INCLUDE_DIRS}/GeographicLib/Config.h")
endif()

if(NOT EXISTS ${GeographicLib_LIBRARIES})
    message(FATAL_ERROR "GeographicLib library not found at ${GeographicLib_LIBRARIES}")
endif()

include_directories(
 include/
 ${EIGEN3_INCLUDE_DIR}
 ${catkin_INCLUDE_DIRS}
 ${GeographicLib_INCLUDE_DIRS}
 ${fmt_INCLUDE_DIRS}
)

# Add library
add_library(${PROJECT_NAME}
  src/eskf.cpp
  src/fusion_node.cpp
)
file(GLOB SOURCES "src/*.cpp")

# Add executable
add_executable(run_awv_fusion ${SOURCES})

target_link_libraries(run_awv_fusion
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  fmt::fmt
  Eigen3::Eigen
  ${GeographicLib_LIBRARIES}
)