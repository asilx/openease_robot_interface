# rosbridge2cpp [![Build Status](https://travis-ci.org/Sanic/travis-cpp11-test.svg?branch=master)](https://travis-ci.org/Sanic/travis-cpp11-test)
A C++11 library to interface ROS via rosbridge

This library can be used to talk to [ROS](http://www.ros.org/) via [rosbridge](http://wiki.ros.org/rosbridge_suite).
It enables you to communicate with ROS even if you don't have the full ROS Stack on your machine available (for example, when you are using Windows).
The network communication of this library is abstracted from a specific network implementation.
This abstraction allows you to use this library in different contexts, for example in Windows Applications or even in Game Engines like [Unreal](https://www.unrealengine.com/).

Please note that this library is in an early development stage and features are added as needed.

## Compiling the library
After cloning the repo, change into that directory and execute:
```
mkdir build
cd build
cmake .. # or 'cmake .. -Dtest=on' if you build the unit tests
make 
```

## Usage
Checkout [src/client/client.cpp](src/client/client.cpp) for an example implementation based on UNIX sockets.
On the server-side, please ensure that you're starting the TCP variant of the rosbridge server.
Websockets are NOT supported.

## Running the unit tests
Please ensure that the you executed cmake with '-Dtest=on' before you continue.
When the library and the unit tests are compiled, execute the following commands on a machine running ROS to setup a minimal testing environment:
```
roslaunch rosbridge_server rosbridge_tcp.launch
rostopic pub /test std_msgs/String a5424890996794277159554918
rosrun rospy_tutorials add_two_ints_server
```
Write down the IP address of the machine where you executed these commands mentally and change to your 'build/' directory.
Call the Unit Tests like this:
```
export rosb2_cpp_ip=THE_IP_ADDRESS_OF_YOUR_ROS_MACHINE; export rosb2_cpp_port=9090; ./runUnitTests
```

