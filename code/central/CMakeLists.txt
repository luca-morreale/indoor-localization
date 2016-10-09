cmake_minimum_required (VERSION 2.8)
project(ITT_centralserver)

SET(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )

include_directories("${PROJECT_SOURCE_DIR}/include")
include(CTest)


set(CLIENT_SRC
   ${PROJECT_SOURCE_DIR}/client.cpp
)

set(CLIENT_H
   ${PROJECT_SOURCE_DIR}/include/client.h
)


add_executable(central main.cpp ${CLIENT_SRC} ${CLIENT_H})

target_link_libraries(central jsoncpp pthread)


#enable_testing()
#add_subdirectory(test)