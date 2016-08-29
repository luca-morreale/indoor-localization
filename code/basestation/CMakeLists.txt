cmake_minimum_required (VERSION 2.8)
project(ITT_basestation)

SET(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11")
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )


include_directories("${PROJECT_SOURCE_DIR}/include")



set(SOURCE
   ${PROJECT_SOURCE_DIR}/data_converter.cpp
)
set(HEADERS
   ${PROJECT_SOURCE_DIR}/include/data_converter.h
)



add_executable(serial serial.cpp ${SOURCE} ${HEADERS})
#add_executable(server server.cpp ${SOURCE} ${HEADERS})

target_link_libraries(serial jsoncpp)
target_link_libraries(serial wiringPi)
target_link_libraries(serial pthread)

#target_link_libraries(server jsoncpp)
#target_link_libraries(server wiringPi)
#target_link_libraries(server pthread)