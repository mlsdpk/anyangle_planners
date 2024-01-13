find_package(OpenCV REQUIRED)
find_package(grid_map_core REQUIRED)

add_library(anyangle_planners_map
    envs/grid_map.cpp
    environment.cpp)

target_include_directories(anyangle_planners_map
    PUBLIC
        ${CMAKE_SOURCE_DIR}/thirdparty
        ${PROJECT_SOURCE_DIR}/include
        ${grid_map_core_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS})

target_link_libraries(anyangle_planners_map 
    PRIVATE 
        ${grid_map_core_LIBRARIES}
        ${OpenCV_LIBRARIES})