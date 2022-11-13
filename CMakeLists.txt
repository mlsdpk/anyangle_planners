cmake_minimum_required(VERSION 3.6.0)
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/project-meta-info.in")

project(
    anyangle_planners
    VERSION ${project_version}
    DESCRIPTION ${project_description})

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS)
catkin_package()

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

# install the generated files
install(
    FILES 
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/cmake/${PROJECT_NAME})

# install headers
install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING
    PATTERN *.h
    PATTERN *.hpp)