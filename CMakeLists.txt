cmake_minimum_required(VERSION 2.8.8)

# ---[ Solution name
project(TSDF C CXX)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(tsdf tsdf.cpp)
target_link_libraries(tsdf ${PCL_LIBRARIES})

install(TARGETS tsdf RUNTIME DESTINATION bin COMPONENT main)
install(FILES ${srcs} DESTINATION app COMPONENT main)
