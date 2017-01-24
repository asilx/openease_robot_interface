#pragma once
#include <functional>

#include "rapidjson/document.h"

#include "messages/rosbridge_msg.h"
#include "messages/rosbridge_service_response_msg.h"

namespace rosbridge2cpp{ 
  using json = rapidjson::Document;
  typedef std::function<void(json&)> FunVcrJSON;
  typedef std::function<void(ROSBridgeMsg&)> FunVrROSMSG;
  typedef std::function<void(ROSBridgeServiceResponseMsg&)> FunVrROSServiceResponseMsg;
  typedef std::function<json(json&)> FunJSONcrJSON;

  enum class TransportError{ SOCKET_ERROR, CONNECTION_CLOSED };
}
