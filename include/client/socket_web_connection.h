#pragma once
#include <iostream>
#include <string>
#include <thread>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

//#include <curlpp/cURLpp.hpp>
//#include <curlpp/Options.hpp>
#include <stdio.h>
#include <curl/curl.h>

#include <iostream>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "itransport_layer.h"
#include "types.h"
#include "helper.h"
#include "client_wss.hpp"
#include "client_ws.hpp"


typedef SimpleWeb::SocketClient<SimpleWeb::WSS> WssClient;
typedef SimpleWeb::SocketClient<SimpleWeb::WS> WsClient;
using json = rapidjson::Document;
using namespace std;
 
namespace rosbridge2cpp{
  class SocketWebConnection : public ITransportLayer{
    public:
      SocketWebConnection () = default;

      ~SocketWebConnection (){
        std::cout << "Connection Destructor called " << std::endl;
        std::cout << "Closing Socket" << std::endl;
        if( client != NULL) client->~WsClient();
        if( client_secure != NULL) client_secure->~WssClient();
      }

      bool StartUserContainer(std::string web_socket_addr, std::string api_token, std::string certificate_path);
      bool Init(std::string web_socket_addr, int p_port);
      bool Authenticate();
      void Refresher();
      bool SendMessage(std::string data);
      void RegisterIncomingMessageCallback(std::function<void(json&)> fun);
      void RegisterErrorCallback(std::function<void(TransportError)> fun);
      void ReportError(TransportError err);

    private:
      static std::string web_addr_;
      static std::string token_user;
      static std::string certificate_path_;
      static string auth_contents;
      static string curl_buffer;

      bool callback_function_defined_ = false;
      bool is_socket_opened = false;
      std::function<void(json&)> incoming_message_callback_;
      std::function<void(TransportError)> error_callback_ = nullptr;
      WssClient *client_secure;
      WsClient *client;
      bool is_secure = false;

      void handler_incoming(const string&);
      void refresh_request();
      static void curl_request(const string&);
      static size_t handle_data(void *ptr, size_t size, size_t nmemb, void *stream); 
  };
}
