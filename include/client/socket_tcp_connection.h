#pragma once
#include <iostream>
#include <string>
#include <thread>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include "rapidjson/document.h"

#include "itransport_layer.h"
#include "types.h"
#include "helper.h"

using json = rapidjson::Document;
 
namespace rosbridge2cpp{
  class SocketTCPConnection : public ITransportLayer{
    public:
      SocketTCPConnection () = default;

      ~SocketTCPConnection (){
        std::cout << "Connection Destructor called " << std::endl;
        std::cout << "Closing Socket" << std::endl;
        if(sock_ >= 0)
          close(sock_);
        terminate_receiver_thread_ = true;
        std::cout << "Waiting for Thread to join" << std::endl;
        if(receiver_thread_set_up_){
          std::cout << "Thread is set up : Calling .join() on it" << std::endl;
          // std::cout << "Joinable? : " << receiverThread.joinable() << std::endl;
          receiver_thread_.join(); // Wait for the receiver thread to finish
          std::cout << "join() in Connection Destructor done" << std::endl;
        }else{
          std::cout << "receiverThread hasn't been set up. Skipping join() on it" << std::endl;
        }
      }

      // A callback that just outputs received messages
      void static messageCallback(const json &message){
        std::string str_repr = Helper::get_string_from_rapidjson(message);
        std::cout << "Message received: " << str_repr << std::endl;
      }


      bool Init(std::string p_ip_addr, int p_port);
      bool SendMessage(std::string data);
      int ReceiverThreadFunction();
      void RegisterIncomingMessageCallback(std::function<void(json&)> fun);
      void RegisterErrorCallback(std::function<void(TransportError)> fun);
      void ReportError(TransportError err);

    private:
      std::string ip_addr_;
      int port_;

      int sock_ = socket(AF_INET , SOCK_STREAM , 0);
      struct sockaddr_in connect_to_;
      std::thread receiver_thread_;
      bool terminate_receiver_thread_ = false;
      bool receiver_thread_set_up_ = false;
      bool callback_function_defined_ = false;
      std::function<void(json&)> incoming_message_callback_;
      std::function<void(TransportError)> error_callback_ = nullptr;
  };
}
