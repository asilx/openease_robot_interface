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

          FRAGMENT, // not implemented currently
          PNG, // not implemented currently
          SET_LEVEL, // not implemented currently
          STATUS, // not implemented currently
          AUTH, // not implemented currently
          ADVERTISE, 
          UNADVERTISE, 
          PUBLISH, 
          SUBSCRIBE, 
          UNSUBSCRIBE, 
          ADVERTISE_SERVICE, 
          UNADVERTISE_SERVICE, 
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
    {"advertise_service", ADVERTISE_SERVICE},
    {"unadvertise_service", UNADVERTISE_SERVICE},
    {"call_service", CALL_SERVICE},
    {"service_response", SERVICE_RESPONSE}
  };

  ROSBridgeMsg () = default;

  // This method can be used to parse incoming ROSBridge messages in order
  // to fill the class variables from the wire representation (for example, JSON or BSON)
  //
  // Returns true if a 'op' field could be found in the given data package
  //
  // This method will care about the 'op' and 'id' key
  // 'op' is mandatory, while 'id' is optional
  bool FromJSON(const rapidjson::Document &data){
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

    if (!data.HasMember("id"))
      return true; // return true, because id is only optional

    id_ = data["id"].GetString();

    return true;
  }
  virtual ~ROSBridgeMsg () = default;

  OpCode op_ = OPCODE_UNDEFINED;
  std::string id_ = "";


private:
  /* data */
};
