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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#add_definitions(-std=c++11)
add_definitions(-DRAPIDJSON_HAS_STDSTRING=1)
add_definitions(-DBSON_COMPILATION)

set (SOURCE_DIR "${PROJECT_SOURCE_DIR}/")

set (SOURCES
   ${SOURCE_DIR}/libbson/bson/bcon.c
   ${SOURCE_DIR}/libbson/bson/bson.c
   ${SOURCE_DIR}/libbson/bson/bson-atomic.c
   ${SOURCE_DIR}/libbson/bson/bson-clock.c
   ${SOURCE_DIR}/libbson/bson/bson-context.c
   ${SOURCE_DIR}/libbson/bson/bson-decimal128.c
   ${SOURCE_DIR}/libbson/bson/bson-error.c
   ${SOURCE_DIR}/libbson/bson/bson-iso8601.c
   ${SOURCE_DIR}/libbson/bson/bson-iter.c
   ${SOURCE_DIR}/libbson/bson/bson-json.c
   ${SOURCE_DIR}/libbson/bson/bson-keys.c
   ${SOURCE_DIR}/libbson/bson/bson-md5.c
   ${SOURCE_DIR}/libbson/bson/bson-memory.c
   ${SOURCE_DIR}/libbson/bson/bson-oid.c
   ${SOURCE_DIR}/libbson/bson/bson-reader.c
   ${SOURCE_DIR}/libbson/bson/bson-string.c
   ${SOURCE_DIR}/libbson/bson/bson-timegm.c
   ${SOURCE_DIR}/libbson/bson/bson-utf8.c
   ${SOURCE_DIR}/libbson/bson/bson-value.c
   ${SOURCE_DIR}/libbson/bson/bson-version-functions.c
   ${SOURCE_DIR}/libbson/bson/bson-writer.c
   ${SOURCE_DIR}/libbson/yajl/yajl_alloc.c
   ${SOURCE_DIR}/libbson/yajl/yajl_buf.c
   ${SOURCE_DIR}/libbson/yajl/yajl.c
   ${SOURCE_DIR}/libbson/yajl/yajl_encode.c
   ${SOURCE_DIR}/libbson/yajl/yajl_gen.c
   ${SOURCE_DIR}/libbson/yajl/yajl_lex.c
   ${SOURCE_DIR}/libbson/yajl/yajl_parser.c
   ${SOURCE_DIR}/libbson/yajl/yajl_tree.c
   ${SOURCE_DIR}/libbson/yajl/yajl_version.c

   ${SOURCE_DIR}/src/client/socket_tcp_connection.cpp 
   ${SOURCE_DIR}/src/ros_bridge.cpp 
   ${SOURCE_DIR}/src/ros_topic.cpp 
   ${SOURCE_DIR}/src/ros_service.cpp 
   ${SOURCE_DIR}/src/ros_tf_broadcaster.cpp 
   ${SOURCE_DIR}/src/ros_time.cpp 
)

INCLUDE_DIRECTORIES( libbson/bson libbson/ include)

#ADD_EXECUTABLE( rosbridge2cpp-client
#	src/client/client.cpp 
#	src/client/socket_tcp_connection.cpp 
#	src/ros_bridge.cpp 
#	src/ros_topic.cpp 
#	src/ros_service.cpp 
#	src/ros_tf_broadcaster.cpp 
#	src/ros_time.cpp 
#)

#ADD_LIBRARY( rosbridge2cpp SHARED
#	src/client/socket_tcp_connection.cpp 
#	src/ros_bridge.cpp 
#	src/ros_topic.cpp 
#	src/ros_service.cpp 
#	src/ros_tf_broadcaster.cpp 
#	src/ros_time.cpp 
#)
ADD_LIBRARY( rosbridge2cpp SHARED ${SOURCES})

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
