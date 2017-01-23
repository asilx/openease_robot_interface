#pragma once

#include <iostream>

#include "messages/rosbridge_msg.h"

class ROSBridgeSubscribeMsg : public ROSBridgeMsg{
public:
  ROSBridgeSubscribeMsg () : ROSBridgeMsg(){
    
  }

  ROSBridgeSubscribeMsg (bool init_opcode) : ROSBridgeMsg(){
    if(init_opcode)
      op_ = ROSBridgeMsg::SUBSCRIBE;
  }

  virtual ~ROSBridgeSubscribeMsg () = default;


  // Subscribe messages will never be received from the client
  // So we don't need to fill this instance from JSON or other wire-level representations
  bool FromJSON(const rapidjson::Document &data) = delete;

  std::string topic_;
  std::string type_;
  int queue_length_ = 0;
  int throttle_rate_ = 0;
  std::string compression_ = "none";
private:
  /* data */
};
