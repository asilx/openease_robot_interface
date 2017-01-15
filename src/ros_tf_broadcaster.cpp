#include "ros_tf_broadcaster.h"

namespace rosbridge2cpp{
  void ROSTFBroadcaster::SendTransform(json &geometry_msgs_transformstamped_msg){
    rapidjson::Document tf_message;
    auto &allocator = tf_message.GetAllocator();
    tf_message.SetObject();
    rapidjson::Value transform_array;
    transform_array.SetArray();

    // rapidjson::Value transform; transform.SetObject();
    transform_array.PushBack(geometry_msgs_transformstamped_msg, allocator);

    tf_message.AddMember("transforms", transform_array, allocator);

    tf_topic_.Publish(tf_message);
  }
}
