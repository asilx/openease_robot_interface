#pragma once

#include <iostream>

#include "messages/rosbridge_msg.h"

class ROSBridgeCallServiceMsg : public ROSBridgeMsg{
public:
  ROSBridgeCallServiceMsg () : ROSBridgeMsg(){
    
  }

  ROSBridgeCallServiceMsg (bool init_opcode) : ROSBridgeMsg(){
    if(init_opcode)
      op_ = ROSBridgeMsg::CALL_SERVICE;
  }

  virtual ~ROSBridgeCallServiceMsg (){
    if(args_bson_!=nullptr)
      bson_destroy(args_bson_);
  }

  // Warning: This conversion moves the 'args' field
  // out of the given JSON data into this class
  // 'args' will become null afterwards.
  //
  // This method parses the "service" and "args" fields from 
  // incoming publish messages into this class
  bool FromJSON(rapidjson::Document &data){
    if(!ROSBridgeMsg::FromJSON(data))
      return false;

    if (!data.HasMember("service")){
      std::cerr << "[ROSBridgeCallServiceMsg] Received 'call_service' message without 'service' field." <<std::endl;
      return false; 
    }

    service_ = data["service"].GetString();

    if (!data.HasMember("args")){
      return true;  // return true, since args is optional. Other parameters will not be set right now
    }

    args_json_ = data["args"];

    return true;
  }

  rapidjson::Document ToJSON(rapidjson::Document::AllocatorType& alloc){
    rapidjson::Document d(rapidjson::kObjectType);
    d.AddMember("op",getOpCodeString(),alloc);

    add_if_value_changed(d, alloc, "id", id_);
    add_if_value_changed(d, alloc, "service", service_);

    if( !args_json_.IsNull() )
      d.AddMember("args", args_json_, alloc);
    return d;
  }

  void ToBSON(bson_t &bson){
    BSON_APPEND_UTF8 (&bson, "op", getOpCodeString().c_str());
    add_if_value_changed(bson, "id", id_);

    add_if_value_changed(bson, "service", service_);
    if( args_bson_ != nullptr){
      if (!BSON_APPEND_DOCUMENT (&bson, "args",args_bson_))
        std::cerr << "Error while appending 'args' bson to messge BSON" << std::endl;
    }
  }
  

  std::string service_;
  // The json data in the different wire-level representations
  rapidjson::Value args_json_;
  bson_t *args_bson_ = nullptr;

private:
  /* data */
};
