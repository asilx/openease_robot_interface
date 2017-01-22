#pragma once

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <unordered_map>
 

/* 
 * The base class for all ROSBridge messages
 *
 * Incoming Messages will be parsed to this class 
 */

class ROSBridgeMsg {
public:
  enum OpCode{
          OPCODE_UNDEFINED, // Default value, before parsing

          FRAGMENT, 
          PNG, 
          SET_LEVEL, 
          STATUS, 
          AUTH, 
          ADVERTISE, 
          UNADVERTISE, 
          PUBLISH, 
          SUBSCRIBE, 
          UNSUBSCRIBE, 
          CALL_SERVICE, 
          SERVICE_RESPONSE
  };

  std::unordered_map<std::string, OpCode> op_code_mapping = { 
    {"fragment", FRAGMENT}, 
    {"png", PNG},
    {"set_level", PNG},
    {"status", STATUS},
    {"auth", AUTH},
    {"advertise", ADVERTISE},
    {"unadvertise", UNADVERTISE},
    {"publish", PUBLISH},
    {"subscribe", SUBSCRIBE},
    {"unsubscribe", UNSUBSCRIBE},
    {"call_service", CALL_SERVICE},
    {"service_response", SERVICE_RESPONSE}
  };

  ROSBridgeMsg () = default;

  // Returns true if parsing was OK
  bool FromJSON(rapidjson::Document data){
    if (!data.HasMember("op")){
      std::cerr << "[ROSBridgeMsg] Received message without 'op' field" <<std::endl;
      return false;
    }

    std::string op_code = data["op"].GetString();
    auto mapping_iterator = op_code_mapping.find(op_code);
    if(mapping_iterator == op_code_mapping.end()){
      std::cerr << "[ROSBridgeMsg] Received message with invalid 'op' field: " << op_code <<std::endl;
      return false;
    }

    op_ = mapping_iterator->second;


    return true;
  }
  virtual ~ROSBridgeMsg () = default;

  OpCode op_ = OPCODE_UNDEFINED;
  std::string id_ = "";


private:
  /* data */
};
