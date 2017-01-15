#pragma once

#include "rapidjson/document.h"

#include "ros_bridge.h"
#include "ros_topic.h"

using json = rapidjson::Document;

namespace rosbridge2cpp{
  class ROSTFBroadcaster {
    public:
      ROSTFBroadcaster (ROSBridge &ros) : ros_(ros){
      };
      void SendTransform(json &geometry_msgs_transformstamped_msg);
      // TODO
      // define send_transform for array of Transforms

      ~ROSTFBroadcaster () = default;;

    private:
      ROSBridge &ros_;
      ROSTopic tf_topic_{ros_,"/tf","tf/tfMessage"};
  };
}
