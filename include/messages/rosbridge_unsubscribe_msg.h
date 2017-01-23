#pragma once

#include <iostream>

#include "messages/rosbridge_msg.h"

class ROSBridgeUnsubscribeMsg : public ROSBridgeMsg{
public:
  ROSBridgeUnsubscribeMsg () : ROSBridgeMsg(){
    
  }

  ROSBridgeUnsubscribeMsg (bool init_opcode) : ROSBridgeMsg(){
    if(init_opcode)
      op_ = ROSBridgeMsg::UNSUBSCRIBE;
  }

  virtual ~ROSBridgeUnsubscribeMsg () = default;


  // Unsubscribe messages will never be received from the client
  // So we don't need to fill this instance from JSON or other wire-level representations
  bool FromJSON(const rapidjson::Document &data) = delete;

  std::string topic_;
private:
  /* data */
};
