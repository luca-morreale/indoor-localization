find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

add_definitions(-DBOOST_TEST_DYN_LINK)

add_executable(converter_test test_converter.cpp)

target_link_libraries (converter_test 
						jsoncpp Converter
						${Boost_FILESYSTEM_LIBRARY}
                      	${Boost_SYSTEM_LIBRARY}
                      	${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)

add_test(ConverterTest converter_test)