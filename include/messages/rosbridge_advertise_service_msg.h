#pragma once

#include <iostream>

#include "messages/rosbridge_msg.h"

class ROSBridgeAdvertiseServiceMsg : public ROSBridgeMsg{
public:
  ROSBridgeAdvertiseServiceMsg () : ROSBridgeMsg(){
    
  }

  ROSBridgeAdvertiseServiceMsg (bool init_opcode) : ROSBridgeMsg(){
    if(init_opcode)
      op_ = ROSBridgeMsg::ADVERTISE_SERVICE;
  }

  virtual ~ROSBridgeAdvertiseServiceMsg () = default;

  // Advertise messages will never be received from the client
  // So we don't need to fill this instance from JSON or other wire-level representations
  bool FromJSON(const rapidjson::Document &data) = delete;

  std::string service_;
  std::string type_;
private:
  /* data */
};

