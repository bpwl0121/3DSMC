cmake_minimum_required(VERSION 3.13)

project(exercise_4)

# Options
set(CMAKE_CXX_STANDARD 14)
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Libs CACHE PATH "Path to lib folder")
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")
set(glog_DIR ${LIBRARY_DIR}/Glog/lib/cmake/glog CACHE PATH "Path to installed glog")
set(Ceres_DIR ${LIBRARY_DIR}/Ceres/CMake CACHE PATH "Path to installed Ceres")

find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)

add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")

set(UTILS
    utils/io.h
    utils/points.h
)

add_executable(gaussian ${UTILS} gaussian.cpp)
add_executable(dragon ${UTILS} dragon.cpp)
add_executable(surface ${UTILS} surface.cpp)

target_link_libraries(gaussian ceres)
target_link_libraries(dragon ceres)
target_link_libraries(surface ceres)

