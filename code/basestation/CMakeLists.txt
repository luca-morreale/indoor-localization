cmake_minimum_required (VERSION 2.8)
project(ITT_basestation)

SET(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )

include_directories("${PROJECT_SOURCE_DIR}/include")
include(CTest)


add_library(Converter ${PROJECT_SOURCE_DIR}/data_converter.cpp ${PROJECT_SOURCE_DIR}/include/data_converter.h)

set(SERVER_SRC
   ${PROJECT_SOURCE_DIR}/data_converter.cpp
   ${PROJECT_SOURCE_DIR}/serial_reader.cpp
   ${PROJECT_SOURCE_DIR}/basestation_server.cpp
)

set(SERVER_H
   ${PROJECT_SOURCE_DIR}/include/data_converter.h
   ${PROJECT_SOURCE_DIR}/include/serial_reader.h
   ${PROJECT_SOURCE_DIR}/include/basestation_server.h
)


add_executable(serial serial.cpp ${SERIAL_SRC} ${SERIAL_H})
add_executable(server server.cpp ${SERVER_SRC} ${SERVER_H})

target_link_libraries(Converter jsoncpp pthread)

target_link_libraries(serial wiringPi Converter)

target_link_libraries(server jsoncpp wiringPi pthread Converter)


enable_testing()
add_subdirectory(test)