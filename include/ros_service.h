#pragma once

#include <list>

#include "rapidjson/document.h"

#include "ros_bridge.h"
#include "types.h"
#include "messages/rosbridge_advertise_service_msg.h"
#include "messages/rosbridge_call_service_msg.h"
#include "messages/rosbridge_unadvertise_service_msg.h"

using json = rapidjson::Document;

namespace rosbridge2cpp{
  class ROSService {
    public:
      // TODO: Implement setter of other options
      ROSService (ROSBridge &ros, std::string service_name, std::string service_type) : 
        ros_(ros), service_name_(service_name), service_type_(service_type){
        }

      // Advertise a service and define the request handling callback
      // The given callback will handle all service requests.
      // The signature of the callback is as follows:
      //   - Return Type: json. 
      //   This json variable will contain the Service Response.
      //   The following attributes shall be present:
      //   response["result"] : Indicating success or a failed service call. REQUIRED!
      //                        This variable shall be either a _boolean_ true (for success) or false (for fail).
      //   response["values"] : A list of the values that shall be returned to the caller. This field MAY be omitted.
      //
      //   - Parameter: json. This contains the service request that we've received via ROSBridge.
      void Advertise(FunJSONcrJSON callback);

      // Unadvertise an advertised service
      // Will do nothing if no service has been advertised before in this instance
      void Unadvertise();

      // TODO failedCallback parameter
      // Call a ROS-Service
      // The given callback variable will be called when the service reply
      // has been received by ROSBridge. It will passed the received data to the callback.
      // The whole content of the "request" parameter will be send as the "args"
      // argument of the Service Request
      void CallService(rapidjson::Value &request, FunVcrJSON callback);

      std::string ServiceName(){
        return service_name_;
      }

    private:
      ROSBridge &ros_;
      std::string service_name_;
      std::string service_type_;
      bool is_advertised_ = false;
  };
}
