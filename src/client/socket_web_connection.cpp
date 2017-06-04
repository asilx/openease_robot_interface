#include "client/socket_web_connection.h"

namespace rosbridge2cpp{
  string SocketWebConnection::web_addr_ = "";
  string SocketWebConnection::token_user = "";
  string SocketWebConnection::certificate_path_ = "";
  string SocketWebConnection::auth_contents = "";
  string SocketWebConnection::curl_buffer = "";

  bool SocketWebConnection::StartUserContainer(std::string web_addr, std::string api_token, std::string certificate_path){
     web_addr_ = web_addr;
     token_user = api_token;
     certificate_path_ = certificate_path;
     
     curl_request("/api/v1.0/start_container/" + token_user);
     if (curl_buffer.find("error") == std::string::npos) {
       json j;
       j.Parse(curl_buffer.c_str());
       string url_returned(j["url"].GetString());

       if(web_addr_.substr(0,5) == "https")
       {
         is_secure = true;
         client_secure = new WssClient(url_returned.substr(2), false);
         client_secure->on_open=[this]() {
           is_socket_opened = true;
           SendMessage(auth_contents);           
         };
    
         client_secure->on_close=[](int status, const string& /*reason*/) {
           cout << "Client: Closed connection with status code " << status << endl;
         };
       }
       else
       {
         is_secure = false;
         client = new WsClient(url_returned.substr(2));

         client->on_open=[this]() {
           is_socket_opened = true;
           SendMessage(auth_contents);
         };
    
         client->on_close=[](int status, const string& /*reason*/) {
           cout << "Client: Closed connection with status code " << status << endl;
         };
       }
         
       return true;
     }
     else {
       std::cout << "error while connecting user container" << std::endl;
       web_addr_ = "error!";
       return false;
     }
   
  }

  bool SocketWebConnection::Init(std::string web_socket_addr, int p_port)
  {
      Authenticate();
      SocketWebConnection::Refresher();
      return true;
  }

  bool SocketWebConnection::Authenticate(){
     curl_request("/api/v1.0/auth_by_token/" + token_user);
       
     if (curl_buffer.find("error") == std::string::npos) {
         cout << "Auth received: " << curl_buffer << endl;
         json j_url;
         j_url.Parse(curl_buffer.c_str());
         string mac_address(j_url["mac"].GetString());
         string client_str(j_url["client"].GetString());
         string destination(j_url["dest"].GetString());
         string random(j_url["rand"].GetString());
         int t = j_url["t"].GetInt();
         string level(j_url["level"].GetString());
         int end = j_url["end"].GetInt();

         json j;
         json::AllocatorType& alloc = j.GetAllocator();
         j.SetObject();
         j.AddMember("op", "auth", alloc);
         j.AddMember("mac", mac_address, alloc);
         j.AddMember("client", client_str, alloc);
         j.AddMember("dest", destination, alloc);
         j.AddMember("rand", random, alloc);
         j.AddMember("level", level, alloc);
         j.AddMember("t", t, alloc);
         j.AddMember("end", end, alloc);
         

         rapidjson::StringBuffer buffer;
         rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
         j.Accept(writer); 


         auth_contents = buffer.GetString();
         thread socket_thread([this](){  
            if(is_secure)
            {
              client_secure->on_message=[this](shared_ptr<WssClient::Message> message) {
                handler_incoming(message->string());
              };
              client_secure->start();
            }
            else
            {
              client->on_message=[this](shared_ptr<WsClient::Message> message) {
                handler_incoming(message->string());
              };     
              client->start();
            }
            
         });
         //std::this_thread::sleep_for(std::chrono::milliseconds(1000));         
         socket_thread.detach();
         return true;
       }
     else {
         std::cout << "error while authenticationg user container" << std::endl;
         web_addr_ = "error!";
         return false;
     }
  }

  void SocketWebConnection::Refresher(){
      thread session_refresher_thread([this](){
         while(true){
           std::this_thread::sleep_for(std::chrono::seconds(50));
           refresh_request();
         }
      });
      session_refresher_thread.detach();
  }

  size_t SocketWebConnection::handle_data(void *ptr, size_t size, size_t nmemb, void *stream) 
  { 
     int numbytes = size*nmemb; 
     // The data is not null-terminated, so get the last character, and replace 
     // it with '\0'. 
     char lastchar = *((char *) ptr + numbytes - 1); 
     *((char *) ptr + numbytes - 1) = '\0'; 
     curl_buffer.append((char *)ptr); 
     curl_buffer.append(1,lastchar); 
     *((char *) ptr + numbytes - 1) = lastchar;  // Might not be necessary. 
     return size*nmemb; 
  } 
  void SocketWebConnection::refresh_request()
  {
     curl_request("/api/v1.0/refresh_by_token/" + token_user);
  }

  void SocketWebConnection::RegisterIncomingMessageCallback(std::function<void(json&)> fun){
     callback_function_defined_ = true;
     incoming_message_callback_ = fun;
     
  }

  void SocketWebConnection::RegisterErrorCallback(std::function<void(TransportError)> fun)
  {  
     error_callback_ = fun;
     if(is_secure)
     {
       //See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html, Error Codes for error code meanings
       client_secure->on_error=[&fun](const boost::system::error_code& ec) {
             cout << "Client: Error: " << ec << ", error message: " << ec.message() << endl;

             TransportError tr = TransportError ::CONNECTION_CLOSED;
             if (ec.message().find("error") != std::string::npos)
             {
                tr = TransportError ::CONNECTION_CLOSED;
             }
             else
             {
                tr = TransportError ::SOCKET_ERROR;
             }
             fun(tr);
       };
     }
     else
     {
        client->on_error=[&fun](const boost::system::error_code& ec) {
             cout << "Client: Error: " << ec << ", error message: " << ec.message() << endl;
          
             TransportError tr = TransportError ::CONNECTION_CLOSED;
             if (ec.message().find("error") != std::string::npos)
             {
                tr = TransportError ::CONNECTION_CLOSED;
             }
             else
             {
                tr = TransportError ::SOCKET_ERROR;
             }
             fun(tr);
        };
     }
  }  
  void SocketWebConnection::ReportError(TransportError err){
    if(error_callback_ == nullptr)
      return;
    error_callback_(err);
  }

  bool SocketWebConnection::SendMessage(std::string msg){
    while(!is_socket_opened ) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    cout << "Socket sending: " << msg << endl;
    if(is_secure)
    {    
      auto send_stream=make_shared<WssClient::SendStream>();
      *send_stream << msg;
      client_secure->send(send_stream); 
    }
    else
    {
      auto send_stream=make_shared<WsClient::SendStream>();
      *send_stream << msg;
      client->send(send_stream);
    }
    return true;
  }

  void SocketWebConnection::handler_incoming(const string& message)
  {
     if(message != "")
     { 
        json json_form;
        json_form.Parse(message.c_str());
        incoming_message_callback_(json_form);
     }
  }

  void SocketWebConnection::curl_request(const string& address)
  {
     curl_buffer = "";

     CURLcode res;

     CURL *curl = curl_easy_init();
     if(curl) {
       string web_auth = web_addr_ + address;
       curl_easy_setopt(curl, CURLOPT_URL, web_auth.c_str());
       curl_easy_setopt(curl, CURLOPT_CAINFO, certificate_path_.c_str());
       curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
       curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
       curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
       curl_easy_setopt(curl, CURLOPT_SSLVERSION, CURL_SSLVERSION_TLSv1);
       curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &SocketWebConnection::handle_data);
       curl_easy_setopt(curl, CURLOPT_WRITEDATA, &curl_buffer);
       /* Perform the request, res will get the return code */
       res = curl_easy_perform(curl);
       cout << "[CURL] returned result:" << curl_buffer << endl;
       /* always cleanup */
       curl_easy_cleanup(curl);
     }

  }
}
