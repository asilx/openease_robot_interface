#include <iostream>
#include <string>
#include <cstdlib>
#include <sstream>
#include <chrono>
#include <functional>
#include "rapidjson/document.h"

#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "gtest/gtest.h"

#include "ros_bridge.h"
#include "ros_topic.h"
#include "ros_service.h"
#include "itransport_layer.h"
#include "client/socket_tcp_connection.h"
#include "helper.h"
#include "ros_message_factory.h"
#include "ros_tf_broadcaster.h"

using json = rapidjson::Document;
using namespace rosbridge2cpp;

/*
 *
 * BEFORE RUNNING THESE TESTS, PLEASE MAKE SURE TO EXECUTE THE FOLLOWING COMMANDS 
 * ON THE SERVER WHERE YOU WANT TO RUN THE ROSBRIDGE SERVICE:
 *   - roslaunch rosbridge_server rosbridge_tcp.launch
 *   - rostopic pub /test std_msgs/String a5424890996794277159554918
 *   - rosrun rospy_tutorials add_two_ints_server
 *
 * Important:
 *  Please note that ROSBridge does alot on an active TCP/IP connection to a rosbridge server.
 *  This means that you _MUST_ pass an IP address and PORT to these tests by setting the following
 *  environment variables:
 *    rosb2_cpp_ip = The IP address of the rosbridge server
 *    rosb2_cpp_port = The port of the rosbridge server. Usually, it's 9090.
 */


TEST(IndependentMethod, ResetsToZero) {
	EXPECT_EQ(0,0);
}

/*
 * A class containing different methods that implement 
 * the behavior to test
 */
class TestHandlerMethods {
public:
  TestHandlerMethods () = default;
  ~TestHandlerMethods () = default;

  void publish_subscribe_test_callback(const json &message){
    std::cout << "[Tests] Service handler received: " << Helper::get_string_from_rapidjson(message) << std::endl;
    std::string data = message["data"].GetString();
    if(data=="a5424890996794277159554918"){
      messageReceived = true;
    }
    else
    {
      std::cout << "[Tests] Received value in test message - Maybe from another node publishing on /test?" << std::endl;
    }
  }

  void service_response_callback(const json &message){
    std::string str_repr = Helper::get_string_from_rapidjson(message);
    std::cout << "[Tests] Service response received: " << str_repr << std::endl;
    bool result = message["result"].GetBool();
    ASSERT_TRUE(result);
    int sum = message["values"]["sum"].GetInt();
    ASSERT_EQ(sum, 42);
    serviceResponseReceived = true;
  }

  json const_service_response_forty_two(const json &message){
    std::string str_repr = Helper::get_string_from_rapidjson(message);
    std::cout << "[Tests] Testing Service handler received: " << str_repr << std::endl;

    rapidjson::Document response;
    response.SetObject();
    response.AddMember("result", true, response.GetAllocator());

    rapidjson::Document values;
    values.SetObject();
    values.AddMember("sum", 42, response.GetAllocator()); // Use responses Allocator, because values will be deconstructed after return;

    response.AddMember("values", values, response.GetAllocator());

    std::cout << "[Tests] Sending service response values: " << Helper::get_string_from_rapidjson(response) << std::endl;

    return response;
  }

  bool messageReceived{false};
  bool serviceResponseReceived{false};

};

class ROSBridgeTest : public ::testing::Test {
public:
	ROSBridgeTest() = default;

  virtual void SetUp() {
    const char* env_ip = std::getenv("rosb2_cpp_ip");
    const char* env_port = std::getenv("rosb2_cpp_port");

    if( env_ip == nullptr)
    {
      FAIL() << "Please set the environment Variables 'rosb2_cpp_ip' before executing the tests. See the README.md";
    }
    if( env_port == nullptr )
    {
      FAIL() << "Please set the environment Variables 'rosb2_cpp_port' before executing the tests. See the README.md";
    }

    std::stringstream portStr;
    portStr << env_port;

    unsigned int port;
    portStr >> port;

    ASSERT_TRUE(ros.Init(env_ip, port)) << "Failed to initialize ROSBridge - This may indicate that it's not possible to connect to the ROSbridge Server";
	}

  SocketTCPConnection t;
  ROSBridge ros{t};
};


// Wait for x milliseconds y times
// Before waiting x milliseconds, the given function will be evaluated
// If it returns true, the used loop will break
void wait_for_x_ms_for_y_steps(int x, int y, std::function<bool ()> l){
  for(int i = 0; i < y; i++){
    if(l()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(x));
  }
}


// Test the ROS init
TEST_F(ROSBridgeTest, InitRosBridge) {
}

TEST_F(ROSBridgeTest, TestTopic) {
  ROSTopic test_topic(ros, "/test", "std_msgs/String");

  TestHandlerMethods thm;
  ASSERT_FALSE(thm.messageReceived);
  auto test_callback = std::bind(&TestHandlerMethods::publish_subscribe_test_callback, &thm, std::placeholders::_1);
  test_topic.Subscribe(test_callback);

  rapidjson::Document message;
  message.SetObject();
  message.AddMember("data", "Publish from Unit-Tests", message.GetAllocator());
  test_topic.Publish(message);

  bool testMessageReceived = false;
  auto l = [&testMessageReceived, &thm]() -> bool{ 
    if(thm.messageReceived){
      testMessageReceived = true; 
      return true;
    }
    return false;
     
  };

  wait_for_x_ms_for_y_steps(100, 10, l);
  ASSERT_TRUE(testMessageReceived) << "Didn't receive the topic test message one second after publish";
}

TEST_F(ROSBridgeTest, CallExternalService) {
  TestHandlerMethods thm;
  ASSERT_FALSE(thm.serviceResponseReceived);
  auto test_callback = std::bind(&TestHandlerMethods::service_response_callback, &thm, std::placeholders::_1);

  ROSService test_service_call(ros, "/add_two_ints", "rospy_tutorials/AddTwoInts");

  rapidjson::Document service_params;
  service_params.SetObject();
  service_params.AddMember("a", 1, service_params.GetAllocator());
  service_params.AddMember("b", 41, service_params.GetAllocator());
  
  test_service_call.CallService(service_params, test_callback);

  bool testResponseReceived = false;
  // Wait a second to check if the message has been received
  auto l = [&testResponseReceived, &thm]() -> bool{ 
    if(thm.serviceResponseReceived){
      testResponseReceived = true; 
      return true;
    }
    return false;
     
  };

  wait_for_x_ms_for_y_steps(100, 10, l);
  ASSERT_TRUE(testResponseReceived) << "Didn't receive the service response one second after service request";
}


TEST_F(ROSBridgeTest, CallOwnService) {
  TestHandlerMethods thm;
  ASSERT_FALSE(thm.serviceResponseReceived);

  // Advertise own service
  ROSService test_service_handler(ros, "/rosbridge_testing_service", "rospy_tutorials/AddTwoInts");
  auto service_request_handler = std::bind(&TestHandlerMethods::const_service_response_forty_two, &thm, std::placeholders::_1);
  test_service_handler.Advertise(service_request_handler);


  auto test_callback = std::bind(&TestHandlerMethods::service_response_callback, &thm, std::placeholders::_1);

  ROSService test_service_call(ros, "/rosbridge_testing_service", "rospy_tutorials/AddTwoInts");
  
  rapidjson::Document service_params;
  service_params.SetObject();
  service_params.AddMember("a", 1, service_params.GetAllocator());
  service_params.AddMember("b", 41, service_params.GetAllocator());
  test_service_call.CallService(service_params, test_callback);

  bool testResponseReceived = false;
  // Wait a second to check if the message has been received
  auto l = [&testResponseReceived, &thm]() -> bool{ 
    if(thm.serviceResponseReceived){
      testResponseReceived = true; 
      return true;
    }
    return false;
     
  };

  wait_for_x_ms_for_y_steps(100, 10, l);
  ASSERT_TRUE(testResponseReceived) << "Didn't receive the service response one second after service request";
  test_service_handler.Unadvertise();
}

TEST_F(ROSBridgeTest, TestTFPublish) {
  ROSTFBroadcaster tfb(ros);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // WARNING!
  // Rapidjson has move semantics and the msg part of a published message will be moved to a rapidjson::document in the sending process
  // To send the same message multiple times, you have to recreate or copy it!
  for (int i = 0; i < 2; i++) {
    unsigned long seconds_since_epoch = 
      std::chrono::duration_cast<std::chrono::seconds>
      (std::chrono::system_clock::now().time_since_epoch()).count();
    unsigned long long nanoseconds_since_epoch = 
      std::chrono::duration_cast<std::chrono::nanoseconds>
      (std::chrono::system_clock::now().time_since_epoch()).count();
    unsigned long nanosecond_difference = nanoseconds_since_epoch - (seconds_since_epoch * 1000000000ul);

    json alloc; // A json document that will only be used to allocate memory in ROSMessageFactory
    json single_transform = ROSMessageFactory::geometry_msgs_transformstamped(alloc.GetAllocator());
    single_transform["header"]["seq"].SetInt(0);
    single_transform["header"]["stamp"]["secs"].SetUint64(seconds_since_epoch);
    single_transform["header"]["stamp"]["nsecs"].SetUint64(nanosecond_difference);
    single_transform["header"]["frame_id"].SetString("/world");
    single_transform["child_frame_id"].SetString("/foobar");
    single_transform["transform"]["translation"]["x"].SetDouble(1);
    single_transform["transform"]["translation"]["y"].SetDouble(0);
    single_transform["transform"]["translation"]["z"].SetDouble(0);

    single_transform["transform"]["rotation"]["x"].SetDouble(0);
    single_transform["transform"]["rotation"]["y"].SetDouble(0);
    single_transform["transform"]["rotation"]["z"].SetDouble(0);
    single_transform["transform"]["rotation"]["w"].SetDouble(1);
    tfb.SendTransform(single_transform);

    json second_transform = ROSMessageFactory::geometry_msgs_transformstamped(alloc.GetAllocator());
    second_transform["header"]["seq"].SetInt(0);
    second_transform["header"]["stamp"]["secs"].SetUint64(seconds_since_epoch);
    second_transform["header"]["stamp"]["nsecs"].SetUint64(nanosecond_difference);
    second_transform["header"]["frame_id"].SetString("/world");
    second_transform["child_frame_id"].SetString("/fasel");
    second_transform["transform"]["translation"]["x"].SetDouble(-1);
    second_transform["transform"]["translation"]["y"].SetDouble(0);
    second_transform["transform"]["translation"]["z"].SetDouble(0);

    second_transform["transform"]["rotation"]["x"].SetDouble(0);
    second_transform["transform"]["rotation"]["y"].SetDouble(0);
    second_transform["transform"]["rotation"]["z"].SetDouble(0);
    second_transform["transform"]["rotation"]["w"].SetDouble(1);
    tfb.SendTransform(second_transform);


    // transform_array.PushBack(single_transform, allocator);

    // tf_message.AddMember("transforms", transform_array, allocator);

    // std::cout << Helper::get_string_from_rapidjson(tf_message) << std::endl;

    // // json tf_message_to_send;
    // // tf_message_to_send.CopyFrom(tf_message, tf_message_to_send.GetAllocator());
    // // test_topic.publish(tf_message_to_send);
    // test_topic.publish(tf_message);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  // TODO test for success
}
