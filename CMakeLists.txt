cmake_minimum_required(VERSION 2.8.8)

# ---[ Solution name
project(TSDF C CXX)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(tsdf tsdf.cpp)
add_executable(marching_cube marching_cubes.cpp marching_cubes_table.h)
target_link_libraries(tsdf ${PCL_LIBRARIES})
target_link_libraries(marching_cube ${PCL_LIBRARIES})

install(TARGETS tsdf marching_cube RUNTIME DESTINATION bin COMPONENT main)
install(FILES ${srcs} DESTINATION app COMPONENT main)
