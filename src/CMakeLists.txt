#find_package(VTK REQUIRED)
#include(${VTK_USE_FILE})

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
find_package(LibRealSense REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/tinytoml)

add_executable(snapshot snapshot.cpp)
target_link_libraries(snapshot ${OpenCV_LIBS} ${LIBREALSENSE_LIBRARIES})
set_target_properties(snapshot PROPERTIES
    DEBUG_POSTFIX "d"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

install(TARGETS snapshot RUNTIME DESTINATION bin COMPONENT main)
install(FILES ${srcs} DESTINATION app COMPONENT main)
