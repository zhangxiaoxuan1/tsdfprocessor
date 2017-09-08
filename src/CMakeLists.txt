#find_package(VTK REQUIRED)
#include(${VTK_USE_FILE})

find_package(Threads REQUIRED)
find_package(Poco REQUIRED COMPONENTS Net)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc imgcodecs videoio)

include_directories(${OpenCV_INCLUDE_DIRS} ${Poco_INCLUDE_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/tinytoml)
include_directories(/home/sahar/code/astra/include)
link_directories(/home/sahar/code/astra/build/lib)

add_executable(snapshot snapshot.cpp)
target_link_libraries(snapshot ${OpenCV_LIBS} ${Poco_LIBRARIES} astra astra_core ${CMAKE_THREAD_LIBS_INIT})
set_target_properties(snapshot PROPERTIES
    DEBUG_POSTFIX "d"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

install(TARGETS snapshot RUNTIME DESTINATION bin COMPONENT main)
install(FILES ${srcs} DESTINATION app COMPONENT main)
