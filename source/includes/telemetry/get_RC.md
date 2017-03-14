## Get RC Data

> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

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

Function Definition: int Navigation::position_set(float x, float y, float z, float yaw=0, float tolerance=0, bool relative=false, bool async=false, bool yaw_valid=false, bool body_frame=false)

Arguments:
    x,y,z: Position Setpoint in NED-Frame (in body-frame if body_frame=true)
    yaw: Yaw Setpoint in radians
    yaw_valid: Must be set to true, if yaw setpoint is provided
    tolerance: Acceptance radius in meters, default value=1.0m
    relative: If true, position setpoints relative to current position is sent
    async: If true, asynchronous mode is set
    body_frame: If true, position setpoints are relative with respect to body frame

Returns: For async=true, returns 0 if the command is successfully sent to the vehicle, else returns 1. For async=false, returns 0 if the vehicle reaches given setpoint before timeout=30secs, else returns 1.
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from python.

NotImplemented
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
rostopic echo /flytpod/mavros/rc/in 
```

```cpp

```

```python
# create flyt_python navigation class instance

NotImplemented
```

```cpp--ros
// Please refer to Roscpp documenation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c%2B%2B)
```

```python--ros
from mavros_msgs.msgs import RCIn
# setup a subscriber and associate a callback function which will be called every time topic is updated.
topic_sub = rospy.Subscriber("/namespace/mavros/rc/in"), State, topic_callback)

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
instance of mavros_msgs/RCIn
```

```cpp
instance of mavros_msgs::RCIn
```

```python
[1001,999,1400,1234,1764,1900]
```

```cpp--ros
success: True
```

```python--ros
NotImplemented
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

This API subscribes/polls the input rc channel data. Please see usage information section below before using the API.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTFul, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Response:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    channels | Array of unit16 | Array of PWM data values for channels.
    
    
    
### ROS endpoint:
All the autopilot state / payload data in FlytOS is shared by ROS topics. Onboard topic subscribers in rospy / roscpp can subscribe to these topics. Take a look at roscpp and rospy API definition for response message structure. 

* Type: Ros Topic</br> 
* Name: /namespace/mavros/rc/in</br>
* Response Type: mavros_msgs/RCIn

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

* Channel mapping of the data depends on RC calibration. 