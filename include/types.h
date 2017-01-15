#pragma once
#include <functional>
#include "rapidjson/document.h"

namespace rosbridge2cpp{ 
  using json = rapidjson::Document;
  typedef std::function<void(json&)> FunVcrJSON;
  typedef std::function<json(json&)> FunJSONcrJSON;

  enum class TransportError{ SOCKET_ERROR, CONNECTION_CLOSED };
}
