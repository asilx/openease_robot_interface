#include "ros_service.h"

namespace rosbridge2cpp{
  void ROSService::CallService(json &request, FunVcrJSON callback){
    if(is_advertised_) // You can't use an advertised ROSService instance to call services. 
      return;         // Use a separate instance

    std::string service_call_id = "";
    service_call_id.append("call_service:");
    service_call_id.append(service_name_);
    service_call_id.append(":");
    service_call_id.append(std::to_string(++ros_.id_counter));

    // Register the callback with the given call id in the ROSBridge
    ros_.RegisterServiceCallback(service_call_id, callback);

    rapidjson::Document cmd;
    cmd.SetObject();
    cmd.AddMember("op","call_service", cmd.GetAllocator());
    cmd.AddMember("id",service_call_id, cmd.GetAllocator());
    cmd.AddMember("service",service_name_, cmd.GetAllocator());
    cmd.AddMember("args", request, cmd.GetAllocator());

    ros_.SendMessage(cmd);
  }

  void ROSService::Advertise(FunJSONcrJSON callback){
    if(is_advertised_) 
      return;

    // Register on ROSBridge
    ros_.RegisterServiceRequestCallback(service_name_, callback);

    rapidjson::Document cmd;
    cmd.SetObject();
    cmd.AddMember("op","advertise_service", cmd.GetAllocator());
    cmd.AddMember("service",service_name_, cmd.GetAllocator());
    cmd.AddMember("type", service_type_, cmd.GetAllocator());

    ros_.SendMessage(cmd);

    is_advertised_ = true;


  }
  // Unadvertise an advertised service
  void ROSService::Unadvertise(){
    if(!is_advertised_) 
      return;

    rapidjson::Document cmd;
    cmd.SetObject();
    cmd.AddMember("op","unadvertise_service", cmd.GetAllocator());
    cmd.AddMember("service",service_name_, cmd.GetAllocator());

    ros_.SendMessage(cmd);

    is_advertised_ = false;
  }
}
