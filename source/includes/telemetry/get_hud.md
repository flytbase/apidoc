## Get VFR HUD



> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Topic Name: /<namespace>/mavros/vfr_hud
ROS-Topic Type: mavros_msgs/VFR_HUD

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    float32 airspeed
    float32 groundspeed
    int16 heading
    float32 throttle
    float32 altitude
    float32 climb


```

```cpp
Not Implemented
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from python.

NotImplemented
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Topic
Name: /<namespace>/mavros/mavros_msgs/vfr_hud

Response structure: mavros_msgs/VFR_HUD
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    float32 airspeed
    float32 groundspeed
    int16 heading
    float32 throttle
    float32 altitude
    float32 climb



```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Topic
Name: /<namespace>/mavros/mavros_msgs/vfr_hud

Response structure: mavros_msgs/VFR_HUD
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    float32 airspeed
    float32 groundspeed
    int16 heading
    float32 throttle
    float32 altitude
    float32 climb
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
rostopic echo /flytpod/mavros/mavros_msgs/vfr_hud
```

```cpp
Not Implemented
```

```python
NotImplemented
```

```cpp--ros
// Please refer to Roscpp documenation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c%2B%2B)
```

```python--ros
from mavros_msgs.msgs import VFR_HUD

# setup a subscriber and associate a callback function which will be called every time topic is updated.
topic_sub = rospy.Subscriber("/namespace/mavros/vfr_hud"), VFR_HUD, topic_callback)

# define the callback function which will print the values every time topic is updated
def topic_callback(data):
    airspeed = data.airspeed
    print airspeed

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
  seq: 3664
  stamp: 
    secs: 1489486597
    nsecs: 915001340
  frame_id: ''
airspeed: 0.0
groundspeed: 0.0
heading: 289
throttle: 0.0
altitude: 601.289001465
climb: -0.0
```

```cpp
Not Implemented
```

```python
NotImplemented
```

```cpp--ros
```

```python--ros
instance of mavros_msgs.msgs.VFR_HUD class

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

This API subscribes/polls VFR HUD data.  Please check API usage section below before using API.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTFul, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Response:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    airspeed | float | airspeed in m/s
    groundspeed | float | groundspeed in m/s
    heading | int16 | yaw angle in degrees (NED frame)
    throttle | float | throttle
    altitude | float | altitude
    climb | float | climb

### ROS endpoint:
All the autopilot state / payload data in FlytOS is shared by ROS topics. Onboard topic subscribers in rospy / roscpp can subscribe to these topics. Take a look at roscpp and rospy API definition for response message structure. 

* Type: Ros Topic</br> 
* Name: /namespace/mavros/vfr_hud</br>
* Response Type: mavros_msgs/VFR_HUD

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

* airspeed data is the data from airspeed sensor.