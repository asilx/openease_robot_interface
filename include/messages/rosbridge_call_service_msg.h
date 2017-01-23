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

  virtual ~ROSBridgeCallServiceMsg () = default;

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

  std::string service_;
  // The json data in the different wire-level representations
  rapidjson::Value args_json_;

private:
  /* data */
};
