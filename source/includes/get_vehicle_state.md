# Get Vehicle State


> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

ROS-Topic Name: /<namespace>/flyt/state
ROS-Topic Type: mavros_msgs/State

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    bool connected
    bool armed
    bool guided
    string mode
    uint8 mav_type
    uint8 mav_autopilot
    uint8 mav_sys_status

```

```cpp

```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from python.

# Python API for vehicle state is split into two APIs

# Check arm status
Class: flyt_python.api.navigation
Function Definition: is_armed()
Arguments: None
return: Boolean

# Check vehicle mode
Class: flyt_python.api.navigation
Function Definition: get_vehicle_mode()
Arguments: None
return: string

```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

ROS-Topic Name: /<namespace>/flyt/state
ROS-Topic Type: mavros_msgs/State

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    bool connected
    bool armed
    bool guided
    string mode
    uint8 mav_type
    uint8 mav_autopilot
    uint8 mav_sys_status

```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

ROS-Topic Name: /<namespace>/flyt/state
ROS-Topic Type: mavros_msgs/State

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    bool connected
    bool armed
    bool guided
    string mode
    uint8 mav_type
    uint8 mav_autopilot
    uint8 mav_sys_status
```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/mavros/imu/data_euler'

JSON Response:
{  twist:{
    linear:{
        x: Float,
        y: Float,
        z: FLoat},
    angular:{
        x: Float,
        y: Float,
        z: FLoat}
}}

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/mavros/imu/data_euler',
messageType: 'geometry_msgs/TwistStamped'

Response:
{   twist:{
    linear:{
        x: Float,
        y: Float,
        z: FLoat},
    angular:{
        x: Float,
        y: Float,
        z: FLoat}
}}

```


> Example

```shell
rotopic echo /flytpod/flyt/state
```

```cpp

```

```python
# create flyt_python navigation class instance

from flyt_python import api
drone = api.navigation()
time.sleep(3.0)

# get arm status
print drone.is_armed()
print drone.get_vehicle_mode()

```

```cpp--ros
// Please refer to Roscpp documenation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c%2B%2B)
```

```python--ros
from mavros_msgs.msgs import State
# setup a subscriber and associate a callback function which will be called every time topic is updated.
topic_sub = rospy.Subscriber("/namespace/flyt/state"), State, topic_callback)

# define the callback function which will print the values every time topic is updated
def topic_callback(data):
    mode, is_armed = data.mode, data.armed
    print mode, is_armed

# unsubscribe from a topic
topic_sub.unregister()  # unregister topic subscription
```

```javascript--REST
$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/mavros/imu/data_euler",  
    success: function(data){
           console.log(data);
    }
};


```

```javascript--Websocket
var imuEulerData = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/mavros/imu/data_euler',
    messageType : 'geometry_msgs/TwistStamped'
});

var request = new ROSLIB.ServiceRequest({});

imuEulerData.subscribe(request, function(result) {
    console.log(result.data);
});
```


> Example response

```shell
header: 
  seq: 1666
  stamp: 
    secs: 1489487905
    nsecs: 864919940
  frame_id: ''
connected: True
armed: False
guided: False
mode: RC|MANUAL
mav_type: 2
mav_autopilot: 12
mav_sys_status: 0
```

```cpp

```

```python
True
MANUAL
```

```cpp--ros
```

```python--ros
True MANUAL
```

```javascript--REST
{
    twist:{
    linear:{
        x: Float,
        y: Float,
        z: FLoat},
    angular:{
        x: Float,
        y: Float,
        z: FLoat}
}

```

```javascript--Websocket
{
    twist:{
    linear:{
        x: Float,
        y: Float,
        z: FLoat},
    angular:{
        x: Float,
        y: Float,
        z: FLoat}
}


```



###Description:

This API subscribes/polls the vehicle state data. Please see usage information section below before using the API.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTFul, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Response:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    mode | string | autopilot flight mode e.g. MANUAL, APICTL
    armed | boolean | Vehicle arm status. Armed if True and disarmed if False.
    
    
    
### ROS endpoint:
All the autopilot state / payload data in FlytOS is shared by ROS topics. Onboard topic subscribers in rospy / roscpp can subscribe to these topics. Take a look at roscpp and rospy API definition for response message structure. 

* Type: Ros Topic</br> 
* Name: /namespace/flyt/state</br>
* Response Type: mavros_msgs/State

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice. All RESTful APIs can poll the data. For telemetry mode (continuous data stream) use websocket APIs.

* URL: ````GET http://<ip>/ros/<namespace>/mavros/imu/data_euler````
* JSON Response:
{
    twist:{
    linear:{
        x: Float,
        y: Float,
        z: FLoat},
    angular:{
        x: Float,
        y: Float,
        z: FLoat}
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/mavros/vfr_hud'</br>
* messageType: 'mavros_msgs/VFR_HUD'

### API usage information:

* This API provides mode and arm status.
* All navigation API's work only in Offboard / APICTL mode. So checking the mode before firing mission critical commands is advised.
* This API only allows to read the mode and arm status. 