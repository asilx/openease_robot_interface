#pragma once

#include "rapidjson/document.h"

#include "ros_bridge.h"
#include "ros_topic.h"
#include "helper.h"

using json = rapidjson::Document;

namespace rosbridge2cpp{
  class ROSTFBroadcaster {
    public:
      ROSTFBroadcaster (ROSBridge &ros) : ros_(ros){
      };

      // Send a single transform to /tf
      void SendTransform(json &geometry_msgs_transformstamped_msg);

      // Accepts an json document (where .IsArray() is true) that contains 
      // an array of geometry_msgs_transformstamped messages. 
      void SendTransforms(json &geometry_msgs_transformstamped_array_msg);

      ~ROSTFBroadcaster () = default;

    private:
      ROSBridge &ros_;
      ROSTopic tf_topic_{ros_,"/tf","tf/tfMessage"};
  };
}
