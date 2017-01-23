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
#include "ros_time.h"
#include "ros_service.h"
#include "itransport_layer.h"
#include "client/socket_tcp_connection.h"
#include "helper.h"
#include "ros_message_factory.h"
#include "ros_tf_broadcaster.h"

#include "messages/rosbridge_msg.h"
#include "messages/rosbridge_publish_msg.h"
#include "messages/rosbridge_call_service_msg.h"
#include "messages/rosbridge_service_response_msg.h"

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


//
// A class containing different methods that implement 
// the behavior to test
//
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
  // ROSBridge ros{t,true};
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
  test_topic.Unadvertise();
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
  for (int i = 0; i < 0; i++) {
    ROSTime time = ROSTime::now();

    json alloc; // A json document that will only be used to allocate memory in ROSMessageFactory
    json single_transform = ROSMessageFactory::geometry_msgs_transformstamped(alloc.GetAllocator());
    single_transform["header"]["seq"].SetInt(0);
    single_transform["header"]["stamp"]["secs"].SetUint(time.sec_);
    single_transform["header"]["stamp"]["nsecs"].SetUint(time.nsec_);
    single_transform["header"]["frame_id"].SetString("/world");
    single_transform["child_frame_id"].SetString("/foobar");
    single_transform["transform"]["translation"]["x"].SetDouble(1);
    single_transform["transform"]["translation"]["y"].SetDouble(0);
    single_transform["transform"]["translation"]["z"].SetDouble(0);

    single_transform["transform"]["rotation"]["x"].SetDouble(0);
    single_transform["transform"]["rotation"]["y"].SetDouble(0);
    single_transform["transform"]["rotation"]["z"].SetDouble(0);
    single_transform["transform"]["rotation"]["w"].SetDouble(1);
    // tfb.SendTransform(single_transform);

    json second_transform = ROSMessageFactory::geometry_msgs_transformstamped(alloc.GetAllocator());
    second_transform["header"]["seq"].SetInt(0);
    second_transform["header"]["stamp"]["secs"].SetUint(time.sec_);
    second_transform["header"]["stamp"]["nsecs"].SetUint(time.nsec_);
    second_transform["header"]["frame_id"].SetString("/world");
    second_transform["child_frame_id"].SetString("/fasel");
    second_transform["transform"]["translation"]["x"].SetDouble(-1);
    second_transform["transform"]["translation"]["y"].SetDouble(0);
    second_transform["transform"]["translation"]["z"].SetDouble(0);

    second_transform["transform"]["rotation"]["x"].SetDouble(0);
    second_transform["transform"]["rotation"]["y"].SetDouble(0);
    second_transform["transform"]["rotation"]["z"].SetDouble(0);
    second_transform["transform"]["rotation"]["w"].SetDouble(1);
    // tfb.SendTransform(second_transform);
    //
    json transforms;
    transforms.SetArray();
    transforms.PushBack(single_transform, transforms.GetAllocator());
    transforms.PushBack(second_transform, transforms.GetAllocator());
    tfb.SendTransforms(transforms);


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

typedef unsigned char BYTE;

static const std::string base64_chars = 
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";


// static inline bool is_base64(BYTE c) {
//   return (isalnum(c) || (c == '+') || (c == '/'));
// }

std::string base64_encode(BYTE const* buf, unsigned int bufLen) {
  std::string ret;
  int i = 0;
  int j = 0;
  BYTE char_array_3[3];
  BYTE char_array_4[4];

  while (bufLen--) {
    char_array_3[i++] = *(buf++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for(i = 0; (i <4) ; i++)
        ret += base64_chars[char_array_4[i]];
      i = 0;
    }
  }

  if (i)
  {
    for(j = i; j < 3; j++)
      char_array_3[j] = '\0';

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    char_array_4[3] = char_array_3[2] & 0x3f;

    for (j = 0; (j < i + 1); j++)
      ret += base64_chars[char_array_4[j]];

    while((i++ < 3))
      ret += '=';
  }

  return ret;
}

// Taken from http://stackoverflow.com/questions/180947/base64-decode-snippet-in-c
// static std::string base64_encode(const std::string &in) {
// 
//     std::string out;
// 
//     int val=0, valb=-6;
//     for (unsigned char c : in) {
//         val = (val<<8) + c;
//         valb += 8;
//         while (valb>=0) {
//             out.push_back("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[(val>>valb)&0x3F]);
//             valb-=6;
//         }
//     }
//     if (valb>-6) out.push_back("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[((val<<8)>>(valb+8))&0x3F]);
//     while (out.size()%4) out.push_back('=');
//     return out;
// }

// Taken from http://stackoverflow.com/questions/180947/base64-decode-snippet-in-c
// static std::string base64_decode(const std::string &in) {
// 
//     std::string out;
// 
//     std::vector<int> T(256,-1);
//     for (int i=0; i<64; i++) T["ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[i]] = i; 
// 
//     int val=0, valb=-8;
//     for (unsigned char c : in) {
//         if (T[c] == -1) break;
//         val = (val<<6) + T[c];
//         valb += 6;
//         if (valb>=0) {
//             out.push_back(char((val>>valb)&0xFF));
//             valb-=8;
//         }
//     }
//     return out;
// }


TEST_F(ROSBridgeTest, PublishImage) {
  ROSTopic imagetopic(ros, "/imagetest", "sensor_msgs/Image");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // WARNING!
  // Rapidjson has move semantics and the msg part of a published message will be moved to a rapidjson::document in the sending process
  // To send the same message multiple times, you have to recreate or copy it!
  for (int i = 0; i < 1; i++) {
    ROSTime time = ROSTime::now();

    json alloc; // A json document that will only be used to allocate memory in ROSMessageFactory
    json image_msg = ROSMessageFactory::sensor_msgs_image(alloc.GetAllocator());
    image_msg["header"]["seq"].SetInt(0);
    image_msg["header"]["stamp"]["secs"].SetUint(time.sec_);
    image_msg["header"]["stamp"]["nsecs"].SetUint(time.nsec_);
    image_msg["header"]["frame_id"].SetString("/camera_frame");
    image_msg["height"].SetUint(1);
    image_msg["width"].SetUint(2);
    image_msg["encoding"].SetString("rgb8");
    image_msg["step"].SetUint(6);
    BYTE image[3] = {0,1,2};
    std::string base64_data = base64_encode(((const BYTE*) &image), 3); // 20,20,20

    image_msg["data"].SetString(base64_data.c_str(), base64_data.length(), alloc.GetAllocator());

    std::cout << "JSON for image" << Helper::get_string_from_rapidjson(image_msg) << std::endl;
    imagetopic.Publish(image_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  // TODO test for success
}


/*
 TEST(IndependentMethod, test_binary) {
  SocketTCPConnection t;
  // ROSBridge ros{t, true};
  ROSBridge ros{t};
  ros.Init("192.168.178.61", 9090);

  {
    json d(rapidjson::kObjectType);
    auto &alloc = d.GetAllocator();
    d.AddMember("op","subscribe", alloc);
    d.AddMember("topic","/test", alloc);
    d.AddMember("type","std_msgs/String", alloc);
    ros.SendMessage(d);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  {
    json d(rapidjson::kObjectType);
    auto &alloc = d.GetAllocator();
    d.AddMember("op","advertise", alloc);
    d.AddMember("topic","/test", alloc);
    d.AddMember("type","std_msgs/String", alloc);
    ros.SendMessage(d);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  for (int i = 0; i < 100; i++) {
    {
      json d(rapidjson::kObjectType);
      json message(rapidjson::kObjectType);
      auto &alloc = d.GetAllocator();
      d.AddMember("op","publish", alloc);
      d.AddMember("topic","/test", alloc);
      d.AddMember("type","std_msgs/String", alloc);
      message.AddMember("data","Hi from BSON",alloc);
      d.AddMember("msg",message,alloc);

      ros.SendMessage(d);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

// {"op":"subscribe","id":"subscribe:/test:1","topic":"/test","type":"std_msgs/String","compression":"none","throttle_rate":0,"queue_length":0}
// {"op":"advertise","id":"advertise:/test:2","topic":"/test","type":"std_msgs/String","latch":false,"queue_size":100}
// {"op":"publish","id":"publish:/test:3","topic":"/test","msg":{"data":"Publish from Unit-Tests"},"latch":false}

  
	EXPECT_EQ(0,0);
}
*/

TEST(IndependentMethod, ROSBridgePublishMsgFromJSON) {
  ROSBridgePublishMsg pm;
  json d(rapidjson::kObjectType);
  json message(rapidjson::kObjectType);
  auto &alloc = d.GetAllocator();
  d.AddMember("op","publish", alloc);
  d.AddMember("topic","/test", alloc);
  d.AddMember("type","std_msgs/String", alloc);
  message.AddMember("data","Hi from Test",alloc);
  d.AddMember("msg",message,alloc);
  pm.FromJSON(d);

  std::cout << Helper::get_string_from_rapidjson(d);

  EXPECT_EQ(pm.op_,ROSBridgeMsg::PUBLISH);
  EXPECT_EQ(pm.topic_,"/test");
  EXPECT_EQ(pm.msg_json_["data"].GetString(),"Hi from Test");
}

TEST(IndependentMethod, ROSBridgeCallServiceFromJSON) {
  ROSBridgeCallServiceMsg cs;
  json d(rapidjson::kObjectType);
  json message(rapidjson::kObjectType);
  auto &alloc = d.GetAllocator();
  d.AddMember("op","call_service", alloc);
  d.AddMember("id","testingid", alloc);
  d.AddMember("service","testservice", alloc);
  message.AddMember("param","one",alloc);
  d.AddMember("args",message,alloc);
  cs.FromJSON(d);

  std::cout << Helper::get_string_from_rapidjson(d);

  EXPECT_EQ(cs.op_,ROSBridgeMsg::CALL_SERVICE);
  EXPECT_EQ(cs.id_,"testingid");
  EXPECT_EQ(cs.service_,"testservice");
  EXPECT_EQ(cs.args_json_["param"].GetString(),"one");
}

TEST(IndependentMethod, ROSBridgeServiceResponseFromJSON) {
  ROSBridgeServiceResponseMsg sr;
  json d(rapidjson::kObjectType);
  json message(rapidjson::kObjectType);
  auto &alloc = d.GetAllocator();
  d.AddMember("op","service_response", alloc);
  d.AddMember("id","testingid", alloc);
  d.AddMember("service","testservice", alloc);
  message.AddMember("param","one",alloc);
  d.AddMember("values",message,alloc);
  d.AddMember("result",true,alloc);
  sr.FromJSON(d);

  std::cout << Helper::get_string_from_rapidjson(d);

  ASSERT_EQ(sr.op_,ROSBridgeMsg::SERVICE_RESPONSE);
  ASSERT_EQ(sr.id_,"testingid");
  ASSERT_EQ(sr.service_,"testservice");
  ASSERT_STREQ(sr.values_json_["param"].GetString(),"one");
  ASSERT_EQ(sr.result_,true);
}

TEST(IndependentMethod, ROSBridgeAdvertiseMsgToJSON) {
  ROSBridgeAdvertiseMsg rosmsg(true);
  // a.op_ = ROSBridgeMsg::ADVERTISE;
  rosmsg.id_ = "id";
  rosmsg.topic_ = "topic";
  rosmsg.type_ = "type";

  json alloc;
  json message = rosmsg.ToJSON(alloc.GetAllocator());

  std::cout << Helper::get_string_from_rapidjson(message);

  ASSERT_STREQ(message["op"].GetString(),"advertise");
  ASSERT_STREQ(message["id"].GetString(),"id");
  ASSERT_STREQ(message["topic"].GetString(),"topic");
  ASSERT_STREQ(message["type"].GetString(),"type");
}

TEST(IndependentMethod, ROSBridgeAdvertiseServiceToJSON) {
  ROSBridgeAdvertiseServiceMsg rosmsg(true);
  rosmsg.id_ = "id";
  rosmsg.service_ = "service";
  rosmsg.type_ = "type";

  json alloc;
  json message = rosmsg.ToJSON(alloc.GetAllocator());

  std::cout << Helper::get_string_from_rapidjson(message);

  ASSERT_STREQ(message["op"].GetString(),"advertise_service");
  ASSERT_STREQ(message["id"].GetString(),"id");
  ASSERT_STREQ(message["service"].GetString(),"service");
  ASSERT_STREQ(message["type"].GetString(),"type");
}

TEST(IndependentMethod, ROSBridgeCallServiceToJSON) {
  ROSBridgeCallServiceMsg rosmsg(true);
  rosmsg.id_ = "id";
  rosmsg.service_ = "service";
  
  json alloc;

  rapidjson::Value args(rapidjson::kObjectType);

  args.AddMember("a","1",alloc.GetAllocator());
  args.AddMember("b","2",alloc.GetAllocator());

  rosmsg.args_json_ = args;

  json message = rosmsg.ToJSON(alloc.GetAllocator());

  std::cout << Helper::get_string_from_rapidjson(message);

  ASSERT_STREQ(message["op"].GetString(),"call_service");
  ASSERT_STREQ(message["id"].GetString(),"id");
  ASSERT_STREQ(message["service"].GetString(),"service");
  ASSERT_STREQ(message["args"]["a"].GetString(),"1");
  ASSERT_STREQ(message["args"]["b"].GetString(),"2");
}

TEST(IndependentMethod, ROSBridgePublishToJSON) {
  ROSBridgePublishMsg rosmsg(true);
  rosmsg.id_ = "id";
  rosmsg.topic_ = "topic";
  rosmsg.type_ = "type";
  rosmsg.latch_ = true;
  
  json alloc;

  rapidjson::Value msg(rapidjson::kObjectType);

  msg.AddMember("data","text",alloc.GetAllocator());

  rosmsg.msg_json_ = msg;

  json message = rosmsg.ToJSON(alloc.GetAllocator());

  std::cout << Helper::get_string_from_rapidjson(message);

  ASSERT_STREQ(message["op"].GetString(),"publish");
  ASSERT_STREQ(message["id"].GetString(),"id");
  ASSERT_STREQ(message["topic"].GetString(),"topic");
  ASSERT_STREQ(message["type"].GetString(),"type");
  ASSERT_STREQ(message["msg"]["data"].GetString(),"text");
}

TEST(IndependentMethod, ROSBridgeServiceResponseToJSON) {
  ROSBridgeServiceResponseMsg rosmsg(true);
  rosmsg.id_ = "id";
  rosmsg.service_ = "service";
  rosmsg.result_ = true;
  
  json alloc;

  rapidjson::Value values(rapidjson::kObjectType);

  values.AddMember("a","b",alloc.GetAllocator());

  rosmsg.values_json_ = values;

  json message = rosmsg.ToJSON(alloc.GetAllocator());

  std::cout << Helper::get_string_from_rapidjson(message);

  ASSERT_STREQ(message["op"].GetString(),"service_response");
  ASSERT_STREQ(message["id"].GetString(),"id");
  ASSERT_STREQ(message["service"].GetString(),"service");
  ASSERT_EQ(message["result"].GetBool(),true);
  ASSERT_STREQ(message["values"]["a"].GetString(),"b");
}

TEST(IndependentMethod, ROSBridgeSubscribeToJSON) {
  ROSBridgeSubscribeMsg rosmsg(true);
  rosmsg.id_ = "id";
  rosmsg.topic_ = "topic";
  rosmsg.type_ = "type";
  rosmsg.queue_length_ = 23;
  rosmsg.throttle_rate_ = 42;
  rosmsg.compression_ = "compression";

  json alloc;

  json message = rosmsg.ToJSON(alloc.GetAllocator());

  std::cout << Helper::get_string_from_rapidjson(message);

  ASSERT_STREQ(message["op"].GetString(),"subscribe");
  ASSERT_STREQ(message["id"].GetString(),"id");
  ASSERT_STREQ(message["topic"].GetString(),"topic");
  ASSERT_STREQ(message["type"].GetString(),"type");
  ASSERT_EQ(message["queue_length"].GetInt(),23);
  ASSERT_EQ(message["throttle_rate"].GetInt(),42);
  ASSERT_STREQ(message["compression"].GetString(),"compression");
}

TEST(IndependentMethod, ROSBridgeUnadvertiseToJSON) {
  ROSBridgeUnadvertiseMsg rosmsg(true);
  rosmsg.id_ = "id";
  rosmsg.topic_ = "topic";

  json alloc;

  json message = rosmsg.ToJSON(alloc.GetAllocator());

  std::cout << Helper::get_string_from_rapidjson(message);

  ASSERT_STREQ(message["op"].GetString(),"unadvertise");
  ASSERT_STREQ(message["id"].GetString(),"id");
  ASSERT_STREQ(message["topic"].GetString(),"topic");
}

TEST(IndependentMethod, ROSBridgeUnadvertiseServiceToJSON) {
  ROSBridgeUnadvertiseServiceMsg rosmsg(true);
  rosmsg.id_ = "id";
  rosmsg.service_ = "service";

  json alloc;

  json message = rosmsg.ToJSON(alloc.GetAllocator());

  std::cout << Helper::get_string_from_rapidjson(message);

  ASSERT_STREQ(message["op"].GetString(),"unadvertise_service");
  ASSERT_STREQ(message["id"].GetString(),"id");
  ASSERT_STREQ(message["service"].GetString(),"service");
}

TEST(IndependentMethod, ROSBridgeUnsubscribeToJSON) {
  ROSBridgeUnsubscribeMsg rosmsg(true);
  rosmsg.id_ = "id";
  rosmsg.topic_ = "topic";

  json alloc;

  json message = rosmsg.ToJSON(alloc.GetAllocator());

  std::cout << Helper::get_string_from_rapidjson(message);

  ASSERT_STREQ(message["op"].GetString(),"unsubscribe");
  ASSERT_STREQ(message["id"].GetString(),"id");
  ASSERT_STREQ(message["topic"].GetString(),"topic");
}

