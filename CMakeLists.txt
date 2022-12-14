cmake_minimum_required(VERSION 3.6.0)
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/project-meta-info.in")

project(
    anyangle_planners
    VERSION ${project_version}
    DESCRIPTION ${project_description})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(grid_map_core REQUIRED)

add_executable(benchmark
    src/benchmark.cpp
    src/algorithm/a_star.cpp
    src/algorithm/planner.cpp
    src/algorithm/theta_star.cpp
    src/algorithm/lazy_theta_star.cpp
    src/algorithm/weighted_lazy_theta_star.cpp
    src/map/environment.cpp
    src/map/scenario_loader.cpp)

target_include_directories(benchmark 
    PUBLIC
        ${grid_map_core_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(benchmark 
    PRIVATE 
        ${grid_map_core_LIBRARIES}
        ${OpenCV_LIBRARIES})

# include(CMakePackageConfigHelpers)

# write_basic_package_version_file(
#     ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
#     VERSION ${PROJECT_VERSION}
#     COMPATIBILITY SameMajorVersion)

# install the generated files
# install(
#     FILES 
#         "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
#     DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/cmake/${PROJECT_NAME})

# install headers
# install(
#     DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#     FILES_MATCHING
#     PATTERN *.h
#     PATTERN *.hpp)