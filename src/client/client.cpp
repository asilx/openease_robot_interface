/*
 * This is an example file to get a feeling for the usage of this library.
 * You may also check our unit tests in tests/tests.cpp to see more functions in action.
 *
 * Please note that this example file uses the socket_tcp_connection class to talk
 * to the rosbridge server. socket_tcp_connection relies on traditional unix sockets and may
 * not work on your individual system.
 */
#include <iostream>
#include <thread>
#include <sstream>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"

#include "types.h"
#include "ros_bridge.h"
#include "ros_topic.h"
#include "ros_service.h"
#include "client/socket_tcp_connection.h"

using json = rapidjson::Document;
using namespace rosbridge2cpp;

void connection_error_handler(TransportError err) {
  if(err == TransportError ::CONNECTION_CLOSED)
    std::cout << "Connection closed - You should reinit ROSBridge" << std::endl;
  if(err == TransportError ::SOCKET_ERROR)
    std::cout << "Error on ROSBridge Socket - You should reinit ROSBridge" << std::endl;
}

json service_handler(const json &message){
  std::string str_repr = Helper::get_string_from_rapidjson(message);
  std::cout << "Service handler received: " << str_repr << std::endl;
  
  rapidjson::Document response;
  response.SetObject();
  response.AddMember("result", true, response.GetAllocator());

  rapidjson::Document values;
  values.SetObject();
  values.AddMember("sum", 23, values.GetAllocator());

  response.AddMember("values", values, response.GetAllocator());

  return response;
}

void messageCallback(const json &message){
  std::cout << "Message received: " << Helper::get_string_from_rapidjson(message) << std::endl;
}

void messageCallback2(const json &message){
  std::cout << "[2nd CB] Message received: " << Helper::get_string_from_rapidjson(message) << std::endl;
}

int main()
{
  SocketTCPConnection t;
  t.RegisterErrorCallback(connection_error_handler);

  ROSBridge ros(t);

    ros.Init("127.0.0.1", 9090);

  ROSTopic test_topic(ros, "/test", "std_msgs/String");

  std::ostringstream oss;

  rapidjson::Document message;
  message.SetObject();
  message.AddMember("data","Example from client", message.GetAllocator());

  test_topic.Publish(message);

  test_topic.Subscribe(messageCallback);
  test_topic.Subscribe(messageCallback2);

  FunVcrJSON cb1 = messageCallback;
  FunVcrJSON cb1a = messageCallback;
  FunVcrJSON cb2 = messageCallback2;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  test_topic.Unsubscribe(messageCallback);
  test_topic.Unsubscribe(messageCallback2);

  ROSService test_service_call(ros, "/add_two_ints", "rospy_tutorials/AddTwoInts");
  
  rapidjson::Document service_params;
  service_params.SetObject();
  service_params.AddMember("a",1, service_params.GetAllocator());
  service_params.AddMember("b",1, service_params.GetAllocator());

  test_service_call.CallService(service_params, messageCallback);

  ROSService test_service_handler(ros, "/meow", "rospy_tutorials/AddTwoInts");
  test_service_handler.Advertise(service_handler);

  while(true){
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  
}
