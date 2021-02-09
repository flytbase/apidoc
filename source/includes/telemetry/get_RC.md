## Get RC Data

> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Topic Name: /<namespace>/mavros/rc/in
ROS-Topic Type: mavros_msgs/RCIn

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    uint8 rssi
    uint16[] channels
```

```cpp
// CPP API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from cpp.

Function Definition: sysSubscribe(Navigation::rc_channels,rcChannelCb);

Arguments:
    rc_channels: This argument selects RC channels topic to be subscribed
    rcChannelCb: Callback function for the subscribed RC channel messages

Returns: RC channel info in ros mavros_msgs::RCInConstPtr structure
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    uint8 rssi
    uint16[] channels
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from python.

Class: flyt_python.API.navigation
Function: get_rc_data()

Response: rc_data as described below.
   class rc_data:
    """
    Holds the input rc channel data
    """
    rssi = 0
    channels = []


This API supports single poll mode only.
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

ROS-Topic Name: /<namespace>/mavros/rc/in
ROS-Topic Type: mavros_msgs/RCIn

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    uint8 rssi
    uint16[] channels

```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

ROS-Topic Name: /<namespace>/mavros/rc/in
ROS-Topic Type: mavros_msgs/RCIn

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    uint8 rssi
    uint16[] channels
```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/mavros/rc/in'

JSON Response:
{
    rssi: Int,
    channels: Int[]
}

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/mavros/rc/in',
messageType: 'mavros_msgs/RCIn'

Response:
{
    rssi: Int,
    channels: Int[]
}

```

```python--flyt_python

# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.flyt_python.DroneApiConnector

Function: get_rc_data()

```

> Example

```shell
rostopic echo /flytos/mavros/rc/in 
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
mavros_msgs::RCIn rc_channel;

void rcChannelCb(void *_rc_channel)
{
    rc_channel = * (mavros_msgs::RCIn*)(_rc_channel);
}
nav.sysSubscribe(Navigation::rc_channels,rcChannelCb);

std::cout << rc_channel << std::endl;
```

```python
# create flyt_python navigation class instance
from flyt_python import API
drone = API.navigation()
# wait for interface to initialize
time.sleep(3.0)

# Poll data
rc = drone.get_rc_data()

# Print the data
print rc.rssi, rc.channels
```

```cpp--ros
// Please refer to Roscpp documentation for sample subscriber nodes. http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
```

```python--ros
import rospy
from mavros_msgs.msgs import RCIn
# setup a subscriber and associate a callback function which will be called every time topic is updated.
topic_sub = rospy.Subscriber("/<namespace>/mavros/rc/in"), State, topic_callback)

# define the callback function which will print the values every time topic is updated
def topic_callback(data):
    # print data from first 6 channels
    print data.channels[:6]


# unsubscribe from a topic
topic_sub.unregister()  # unregister topic subscription
```

```javascript--REST
$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/mavros/rc/in",  
    success: function(data){
           console.log(data);
    }
});


```

```javascript--Websocket
var rcData = new ROSLIB.Topic({
    ros : ros,
    name : '/<namespace>/mavros/rc/in',
    messageType : 'mavros_msgs/RCIn'
});

rcData.subscribe(function(message) {
    console.log(message.data);
});
```

```python--flyt_python 
from flyt_python.flyt_python import DroneApiConnector
token = ''                      # Personal Access Token
vehicle_id = ''                 # Vehicle ID

#create an instance of class DroneApiConnector
drone = DroneApiConnector(token,vehicle_id,ip_address='localhost' wait_for_drone_response =True)
drone.connect()
    
drone.get_rc_data()

drone.disconnect()
```

> Example response

```shell
instance of mavros_msgs/RCIn
```

```cpp
instance of mavros_msgs::RCIn
```

```python
instance of class rc_data
```

```cpp--ros
success: True
```

```python--ros
[1001,999,1400,1234,1764,1900]
```

```javascript--REST
{
    rssi: Int,
    channels: Int[]
}

```

```javascript--Websocket
{
    rssi: Int,
    channels: Int[]
}
```

```python--flyt_python
{
    rssi: Int,
    channels: Int[]
}
```

### Description:

This API subscribes/polls the input rc channel data. Please see usage information section below before using the API.

### Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Response:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    channels | Array of unit16 | Array of PWM data values for channels.    

### API usage information:

* Channel mapping of the data depends on RC calibration. 
    
### ROS endpoint:

All the autopilot state / payload data in FlytOS is shared by ROS topics. Onboard topic subscribers in rospy / roscpp can subscribe to these topics. Take a look at roscpp and rospy API definition for response message structure. 

* Type: `Ros Topic`
* Name: `/<namespace>/mavros/rc/in`
* Response Type: `mavros_msgs/RCIn`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice. All RESTful APIs can poll the data. For telemetry mode (continuous data stream) use websocket APIs.

* URL: `GET http://<ip>/ros/<namespace>/mavros/rc/in`
* JSON Response:
`{
    rssi: Int,
    channels: Int[]
}`


### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 

Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace>/mavros/rc/in`
* messageType: `mavros_msgs/RCIn`
