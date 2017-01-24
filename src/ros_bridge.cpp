#include "ros_bridge.h"
#include "ros_topic.h"
#include <bson.h>

namespace rosbridge2cpp{
  bool ROSBridge::SendMessage(std::string data){
    return transport_layer_.SendMessage(data);
  }

  bool ROSBridge::SendMessage(json &data){
    if(bson_only_mode_){
      // going from JSON to BSON
      std::string str_repr = Helper::get_string_from_rapidjson(data); 
      std::cout << "[ROSBridge] serializing from JSON to BSON for: " << str_repr << std::endl;
      // return transport_layer_.SendMessage(data,length);
      
      bson_t bson;
      bson_error_t error;
      if (!bson_init_from_json(&bson, str_repr.c_str(), -1, &error)) {
        printf("bson_init_from_json() failed: %s\n", error.message);
        bson_destroy(&bson);
        return false;
      }
      const uint8_t *bson_data = bson_get_data (&bson);
      uint32_t bson_size = bson.len;
      bool retval = transport_layer_.SendMessage(bson_data,bson_size);
      bson_destroy(&bson);
      return retval;
    }else{
      std::string str_repr = Helper::get_string_from_rapidjson(data); 
      return SendMessage(str_repr);
    }
  }

  bool ROSBridge::SendMessage(ROSBridgeMsg &msg){

    if(bson_only_mode_){
      // // going from JSON to BSON
      // std::string str_repr = Helper::get_string_from_rapidjson(data); 
      // std::cout << "[ROSBridge] serializing from JSON to BSON for: " << str_repr << std::endl;
      // // return transport_layer_.SendMessage(data,length);
      // 
      // bson_t bson;
      // bson_error_t error;
      // if (!bson_init_from_json(&bson, str_repr.c_str(), -1, &error)) {
      //   printf("bson_init_from_json() failed: %s\n", error.message);
      //   bson_destroy(&bson);
      //   return false;
      // }
      // const uint8_t *bson_data = bson_get_data (&bson);
      // uint32_t bson_size = bson.len;
      // bool retval = transport_layer_.SendMessage(bson_data,bson_size);
      // bson_destroy(&bson);
      std::cerr << "Not implemented" << std::endl;
      return false;
    }


    // Convert ROSBridgeMsg to JSON
    json alloc;
    json message = msg.ToJSON(alloc.GetAllocator());

    std::string str_repr = Helper::get_string_from_rapidjson(message); 
    return SendMessage(str_repr);
    // return false;
  }

  void ROSBridge::HandleIncomingPublishMessage(ROSBridgePublishMsg &data){
    //Incoming topic message - dispatch to correct callback
    // if (!data.HasMember("topic")){
    //   std::cerr << "[ROSBridge] Received 'publish' message without 'topic' field." <<std::endl;
    //   return;
    // }

    // std::string incoming_topic_name = data["topic"].GetString();
    std::string incoming_topic_name = data.topic_;
    if ( registered_topic_callbacks_.find(incoming_topic_name) == registered_topic_callbacks_.end()) {
      std::cerr << "[ROSBridge] Received message for topic " << incoming_topic_name << "where no callback has been registered before" <<std::endl;
      return;
    }
    if ( registered_topic_callbacks_.empty()) {
      std::cerr << "[ROSBridge] Received message for topic " << incoming_topic_name << "where no callback is currently registered" <<std::endl;
      return;
    }

    // if (!data.HasMember("msg")){
    if ( data.msg_json_.IsNull()){
      std::cerr << "[ROSBridge] Received message for topic " << incoming_topic_name << ", but 'msg' field is missing. Aborting" <<std::endl;
      return;
    }

    // Iterate over all registered callbacks for the given topic
    for(auto topic_callback : registered_topic_callbacks_.find(incoming_topic_name)->second){
      // json msg;
      // msg.CopyFrom(data["msg"], msg.GetAllocator());
      // msg.CopyFrom(data.msg_json_, msg.GetAllocator());
      // topic_callback(msg);
      topic_callback(data);
    }
    return;
  }

  void ROSBridge::HandleIncomingServiceResponseMessage(ROSBridgeServiceResponseMsg &data){
    // if (!data.HasMember("service")){
    //   std::cerr << "[ROSBridge] Received 'service_response' message without 'service' field." <<std::endl;
    //   return;
    // }

    // if (!data.HasMember("id")){
    //   std::cerr << "[ROSBridge] Received 'service_response' message without 'id' field. Discarding message." <<std::endl;
    //   return;
    // }

    // std::string incoming_service_id = data["id"].GetString();
    std::string incoming_service_id = data.id_;

    auto service_response_callback_it = registered_service_callbacks_.find(incoming_service_id);

    if ( service_response_callback_it == registered_service_callbacks_.end()) {
      std::cerr << "[ROSBridge] Received response for service id " << incoming_service_id << "where no callback has been registered before" <<std::endl;
      return;
    }

    // Execute the callback for the given service id
    service_response_callback_it->second(data);

    // Delete the callback.
    // Every call_service will create a new id
    registered_service_callbacks_.erase(service_response_callback_it);

  }

  void ROSBridge::HandleIncomingServiceRequestMessage(std::string id, ROSBridgeCallServiceMsg &data){
    // if (!data.HasMember("service")){
    //   std::cerr << "[ROSBridge] Received 'call_service' message without 'service' field." <<std::endl;
    //   return;
    // }

    // std::string incoming_service = data["service"].GetString();
    std::string incoming_service = data.service_;

    auto service_request_callback_it =  registered_service_request_callbacks_.find(incoming_service);

    if ( service_request_callback_it == registered_service_request_callbacks_.end()) {
      std::cerr << "[ROSBridge] Received service request for service :" << incoming_service << " where no callback has been registered before" <<std::endl;
      return;
    }

    ROSBridgeServiceResponseMsg response(true);
    response.service_ = incoming_service;
    if(id!="")
      response.id_ = id;

    // TODO Switch for bson
    if(bson_only_mode_){
      std::cerr << "HandleIncomingServiceRequestMessage not implemented for BSON" << std::endl;
      return;
    }

    response.values_json_.SetObject();
    rapidjson::Document response_allocator;

    // Execute the callback for the given service id
    service_request_callback_it->second(data,response,response_allocator.GetAllocator());
    // if (!response.HasMember("result")){
    //   std::cerr << "[ROSBridge] The callback handler for " << incoming_service << " doesn't contain a result. Aborting reply" <<std::endl;
    //   // TODO Respond with failed service call?
    //   return;
    // }

    // json full_response;
    // full_response.SetObject();
    // full_response.AddMember("op","service_response", full_response.GetAllocator());
    // full_response.AddMember("service", incoming_service, full_response.GetAllocator());
    // full_response.AddMember("result", response["result"], full_response.GetAllocator());

    // if(id!=""){
    //   full_response.AddMember("id", id, full_response.GetAllocator());
    // }

    // Return values may be omitted.
    // Include them in the response when given.
    // if(response.HasMember("values")){
    //   full_response.AddMember("values", response["values"], full_response.GetAllocator());
    // }//else{
    //   std::cout << "[ROSBridge] Service response for " << incoming_service << " didn't contain any values" <<  std::endl;
    // }

    // std::cout << "[ROSBridge] Full service response: " << Helper::get_string_from_rapidjson(full_response) << std::endl;
    SendMessage(response);
  }

  void ROSBridge::IncomingMessageCallback(json &data){
    std::string str_repr = Helper::get_string_from_rapidjson(data);
    std::cout << "[ROSBridge] Received data: " << str_repr << std::endl;


    // Check the message type and dispatch the message properly
    //
    // Incoming Topic messages
    if(std::string(data["op"].GetString(), data["op"].GetStringLength())== "publish"){
      ROSBridgePublishMsg m;
      if(m.FromJSON(data)){
        HandleIncomingPublishMessage(m);
      }else{
        std::cerr << "Failed to parse publish message into class. Skipping message." << std::endl;
      }
    }

    // Service responses for service we called earlier
    if(std::string(data["op"].GetString(), data["op"].GetStringLength()) == "service_response"){
      ROSBridgeServiceResponseMsg m;
      m.FromJSON(data);
      if(m.FromJSON(data)){
        HandleIncomingServiceResponseMessage(m);
      }else{
        std::cerr << "Failed to parse service_response message into class. Skipping message." << std::endl;
      }
    }

    // Service Requests to a service that we advertised in ROSService
    if(std::string(data["op"].GetString(), data["op"].GetStringLength()) == "call_service"){
      ROSBridgeCallServiceMsg m;
      m.FromJSON(data);
      // std::string id = "";
      // if(data.HasMember("id"))
      //   id = data["id"].GetString();
      HandleIncomingServiceRequestMessage(m.id_, m);
    }


  }

  bool ROSBridge::Init(std::string ip_addr, int port){
    std::function<void(json&)> fun = std::bind(&ROSBridge::IncomingMessageCallback, this, std::placeholders::_1);
    if(bson_only_mode_)
      transport_layer_.SetTransportMode(ITransportLayer::BSON);

    transport_layer_.RegisterIncomingMessageCallback(fun);
    return transport_layer_.Init(ip_addr,port);
  }

  void ROSBridge::RegisterTopicCallback(std::string topic_name, FunVrROSPublishMsg fun){
    registered_topic_callbacks_[topic_name].push_back(fun);
  }

  void ROSBridge::RegisterServiceCallback(std::string service_call_id, FunVrROSServiceResponseMsg fun){
    registered_service_callbacks_[service_call_id] = fun;
  }

  void ROSBridge::RegisterServiceRequestCallback(std::string service_name, FunVrROSCallServiceMsgrROSServiceResponseMsgrAllocator fun){
    registered_service_request_callbacks_[service_name] = fun;
  }


  bool ROSBridge::UnregisterTopicCallback(std::string topic_name, FunVrROSPublishMsg fun){
    if ( registered_topic_callbacks_.find(topic_name) == registered_topic_callbacks_.end()) {
      std::cerr << "[ROSBridge] UnregisterTopicCallback called but given topic name" << topic_name << " not in map." <<std::endl;
      return false;
    }
    if ( registered_topic_callbacks_.empty()) {
      std::cerr << "[ROSBridge] UnregisterTopicCallback called but given topic name" << topic_name << " is empty in map." <<std::endl;
      return false;
    }

    std::list<FunVrROSPublishMsg> &r_list_of_callbacks = registered_topic_callbacks_.find(topic_name)->second;

    for(std::list<FunVrROSPublishMsg>::iterator topic_callback_it = r_list_of_callbacks.begin(); 
        topic_callback_it!= r_list_of_callbacks.end();
        ++topic_callback_it){
      if(get_address(*topic_callback_it) == get_address(fun)){
        std::cout << "[ROSBridge] Found CB in UnregisterTopicCallback. Deleting it ... " << std::endl;
        r_list_of_callbacks.erase(topic_callback_it);
        return true;
      }
    }
    return false;
  }
}
