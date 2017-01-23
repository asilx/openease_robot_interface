#pragma once

#include <iostream>

#include "messages/rosbridge_msg.h"

class ROSBridgeAdvertiseMsg : public ROSBridgeMsg{
public:
  ROSBridgeAdvertiseMsg () : ROSBridgeMsg(){
    
  }

  ROSBridgeAdvertiseMsg (bool init_opcode) : ROSBridgeMsg(){
    if(init_opcode)
      op_ = ROSBridgeMsg::ADVERTISE;
  }

  virtual ~ROSBridgeAdvertiseMsg () = default;


  // Advertise messages will never be received from the client
  // So we don't need to fill this instance from JSON or other wire-level representations
  bool FromJSON(const rapidjson::Document &data) = delete;

  std::string topic_;
  std::string type_;
  // std::string compression_ = "none";
  // int throttle_rate_ = 0;
  bool latch_ = false;
  int queue_size_ = 100;
private:
  /* data */
};
