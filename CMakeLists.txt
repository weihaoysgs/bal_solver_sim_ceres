cmake_minimum_required(VERSION 2.8)

project(BA)

set(CMAKE_BUILD_TYPE "Release")

include_directories("./eigen3")

set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories( ${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(bound_adjustment_ceres bound_adjustment_ceres.cpp)
add_executable(BoundleAdjustmentByNode_accelebrate ba_main.cpp)

target_link_libraries(bound_adjustment_ceres ${CERES_LIBRARIES})

target_link_libraries(BoundleAdjustmentByNode_accelebrate ${CERES_LIBRARIES})
