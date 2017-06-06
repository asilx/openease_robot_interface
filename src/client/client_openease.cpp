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
#include "client/socket_web_connection.h"

using json = rapidjson::Document;
using namespace rosbridge2cpp;

string task_id = "";

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

void startTaskCallback(json &message){
  string solution = message["values"]["solution"].GetString();
  json j;
  j.Parse(solution.c_str());
  task_id = j["ActionInst"].GetString();
  std::cout << "Message received: " << Helper::get_string_from_rapidjson(message) << std::endl;
  std::cout << "Created Task: " << task_id << std::endl;
}

void ordinaryCallback(json &message){
  std::cout << "Message received: " << Helper::get_string_from_rapidjson(message) << std::endl;
}

int main()
{
  
  SocketWebConnection t;
  t.StartUserContainer("https://localhost", "6ESR8hTRBepTSj1Vy00kJDJRryUPAqKOHKI8dFwrCxubZfGXCtDHkr6SS8I2bRlP", "/home/asil/mycert.pem");  
  t.RegisterErrorCallback(connection_error_handler);

  ROSBridge ros(t);
  ros.Init("https://localhost", 443);
  //std::this_thread::sleep_for(std::chrono::milliseconds(15000));
  /*ROSTopic test_topic(ros, "/test", "std_msgs/String");
  std::ostringstream oss;

  rapidjson::Document message;
  message.SetObject();
  message.AddMember("data","Example from client", message.GetAllocator());
  test_topic.Subscribe(messageCallback);
  test_topic.Publish(message);

  FunVcrJSON cb1 = messageCallback;
  FunVcrJSON cb1a = messageCallback;*/

  //waiting for json prolog server initilized in the user container
  std::this_thread::sleep_for(std::chrono::seconds(10));
  

  ROSService simple_query_service(ros, "/json_prolog/simple_query", "json_prolog_msgs/PrologQuery");
  rapidjson::Document start_task_query_params;
  start_task_query_params.SetObject();
  start_task_query_params.AddMember("mode",1, start_task_query_params.GetAllocator());
  start_task_query_params.AddMember("id","hgahg6", start_task_query_params.GetAllocator());
  start_task_query_params.AddMember("query","cram_start_action(knowrob:'CRAMAction', 'DummyContext', 1492785072, PA, ActionInst)", start_task_query_params.GetAllocator());

  simple_query_service.CallService(start_task_query_params, ordinaryCallback);

  ROSService next_solution_service(ros, "/json_prolog/next_solution", "json_prolog_msgs/PrologNextSolution");
  
  rapidjson::Document next_solution_params;
  next_solution_params.SetObject();
  next_solution_params.AddMember("id","hgahg6", next_solution_params.GetAllocator());
  next_solution_service.CallService(next_solution_params, startTaskCallback);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  rapidjson::Document end_task_query_params;
  end_task_query_params.SetObject();
  end_task_query_params.AddMember("mode",1, end_task_query_params.GetAllocator());
  end_task_query_params.AddMember("id","hgahg7", end_task_query_params.GetAllocator());
  end_task_query_params.AddMember("query","cram_finish_action('" + task_id + "', 1492785075)", end_task_query_params.GetAllocator());

  simple_query_service.CallService(end_task_query_params, ordinaryCallback);
  
  
  rapidjson::Document next_solution_params2;
  next_solution_params2.SetObject();
  next_solution_params2.AddMember("id","hgahg7", next_solution_params2.GetAllocator());
  next_solution_service.CallService(next_solution_params2, ordinaryCallback);
  
  std::this_thread::sleep_for(std::chrono::seconds(1));

  rapidjson::Document save_query_params;
  save_query_params.SetObject();
  save_query_params.AddMember("mode",1, save_query_params.GetAllocator());
  save_query_params.AddMember("id","hgahg8", save_query_params.GetAllocator());
  save_query_params.AddMember("query","rdf_save('/home/ros/user_data/testxx.owl', [graph('LoggingGraph')])", save_query_params.GetAllocator());

  simple_query_service.CallService(save_query_params, ordinaryCallback);

  
  rapidjson::Document next_solution_params3;
  next_solution_params3.SetObject();
  next_solution_params3.AddMember("id","hgahg8", next_solution_params3.GetAllocator());
  next_solution_service.CallService(next_solution_params3, ordinaryCallback);

  /*ROSService test_service_handler(ros, "/meow", "rospy_tutorials/AddTwoInts");
  test_service_handler.Advertise(service_handler);*/

  while(true){
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  
}
