cmake_minimum_required(VERSION 3.10)

project(bal_solver_sim_ceres)

set(CMAKE_BUILD_TYPE "Debug")

include_directories("./eigen3")

set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories( ${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(bal_solver_sim_ceres)

add_executable(bound_adjustment_ceres bound_adjustment_ceres.cpp)
add_executable(BoundleAdjustmentByNode_accelebrate ba_main.cpp)
add_executable(ba_pnp ba_pnp.cpp)
add_executable(ba_pnp_ceres ba_pnp_ceres.cpp)
target_link_libraries(bound_adjustment_ceres ${CERES_LIBRARIES})
target_link_libraries(BoundleAdjustmentByNode_accelebrate ${CERES_LIBRARIES})
target_link_libraries(ba_pnp ${CERES_LIBRARIES})
target_link_libraries(ba_pnp_ceres ${CERES_LIBRARIES})
