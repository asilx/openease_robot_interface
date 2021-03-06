# Thanks to https://github.com/dmonopoly/gtest-cmake-example/blob/master/CMakeLists.txt
# for providing a basic setup to use gtest in a cmake project
# Another solution would make use of ExternalProject_Add to downloads and installs
# googletest in the build/ dir:
# http://stackoverflow.com/questions/9689183/cmake-googletest/9695234#9695234
# However, this requires an active internet connection


cmake_minimum_required(VERSION 2.8)
option(test "Build all tests." OFF) # Makes boolean 'test' available.
set(PROJECT_NAME rosbridge2cpp)
project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-g -Wall")
add_definitions(-std=c++11)
add_definitions(-DRAPIDJSON_HAS_STDSTRING=1)


FIND_PACKAGE(CURL)
find_package( Boost REQUIRED COMPONENTS program_options regex )
include_directories( ${Boost_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES(${CURL_INCLUDE_DIR})

INCLUDE_DIRECTORIES( include)
ADD_EXECUTABLE( rosbridge2cpp-client
	src/client/client.cpp 
	src/client/socket_tcp_connection.cpp
	src/ros_bridge.cpp 
	src/ros_topic.cpp 
	src/ros_service.cpp 
	src/ros_tf_broadcaster.cpp 
)

ADD_LIBRARY( rosbridge2cpp SHARED
	src/client/socket_tcp_connection.cpp
	src/ros_bridge.cpp 
	src/ros_topic.cpp 
	src/ros_service.cpp 
	src/ros_tf_broadcaster.cpp 
)

ADD_EXECUTABLE( oebridge2cpp-client
	src/client/client_openease.cpp 
        src/client/socket_web_connection.cpp 
	src/ros_bridge.cpp 
	src/ros_topic.cpp 
	src/ros_service.cpp 
	src/ros_tf_broadcaster.cpp 
)

ADD_LIBRARY( oebridge2cpp SHARED
        src/client/socket_web_connection.cpp 
	src/ros_bridge.cpp 
	src/ros_topic.cpp 
	src/ros_service.cpp 
	src/ros_tf_broadcaster.cpp 
)

target_link_libraries(oebridge2cpp-client -lssl -lcrypto -lpthread -lboost_system ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_REGEX_LIBRARY} ${CURL_LIBRARIES} )
################################
# Testing
################################
if (test)
	# This adds another subdirectory, which has 'project(gtest)'.
	add_subdirectory(lib/googletest/googletest)

	enable_testing()

	# Include the gtest library. gtest_SOURCE_DIR is available due to
	# 'project(gtest)' above.
	include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

	##############
	# Unit Tests
	##############
	add_executable(runUnitTests tests/tests.cpp)

	# Standard linking to gtest stuff.
	target_link_libraries(runUnitTests gtest gtest_main rosbridge2cpp)

	# Extra linking for the project.
	#target_link_libraries(runUnitTests project1_lib)

	# This is so you can do 'make test' to see all your tests run, instead of
	# manually running the executable runUnitTests to see those specific tests.
  #add_test(NAME that-test-I-made COMMAND runUnitTests)

	# You can also omit NAME and COMMAND. The second argument could be some other
	# test executable.
	add_test(that-other-test-I-made runUnitTests)
endif()
