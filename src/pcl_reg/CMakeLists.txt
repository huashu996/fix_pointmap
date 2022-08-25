cmake_minimum_required(VERSION 2.8.3)
project(pcl_reg)

add_compile_options(-std=c++11)


# -mavx causes a lot of errors!!
add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  message_filters
)

find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS 
  geometry_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs cv_bridge
  DEPENDS eigen opencv
  INCLUDE_DIRS include
  LIBRARIES ndt_omp
)


###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(ndt_omp
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcl_reg src/pcl_reg.cpp)
target_link_libraries(pcl_reg 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(pcl_reg
  ndt_omp
)
target_link_libraries(pcl_reg
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
)


add_executable(pcl_reg_rgb src/pcl_reg_rgb.cpp)
target_link_libraries(pcl_reg_rgb
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(pcl_reg_rgb
  ndt_omp
)
target_link_libraries(pcl_reg_rgb
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
)


