#pragma once

#include <iostream>

#include "messages/rosbridge_msg.h"

class ROSBridgeUnadvertiseMsg : public ROSBridgeMsg{
public:
  ROSBridgeUnadvertisesg () : ROSBridgeMsg(){
    
  }

  ROSBridgeUnadvertisesg (bool init_opcode) : ROSBridgeMsg(){
    if(init_opcode)
      op_ = ROSBridgeMsg::UNADVERTISE;
  }

  virtual ~ROSBridgeUnadvertiseMsg () = default;


  // Unadvertise messages will never be received from the client
  // So we don't need to fill this instance from JSON or other wire-level representations
  bool FromJSON(const rapidjson::Document &data) = delete;

  std::string topic_;
private:
  /* data */
};

