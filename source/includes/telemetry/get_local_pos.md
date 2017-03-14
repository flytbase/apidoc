## Get Local Position 


> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

ROS-Topic Name: /<namespace>/mavros/imu/local_position/local
ROS-Topic Type: geometry_msgs/TwistStamped, below is its description

#Subscriber response : Euler angles 
Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Twist twist
      geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z

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

Class: flyt_python.api.navigation

Function: get_local_position()

Response: local_position as described below.
    class local_position:
        '''
        Holds fields for local position
        '''
        x = 0.0
        y = 0.0
        z = 0.0
        vx = 0.0
        vy = 0.0
        vz = 0.0
    
This API support single pole mode only.
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Topic
Name: /<namespace>/mavros/local_position/local
Response Type:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Twist twist
      geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z

```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Topic
Name: /<namespace>/mavros/local_position/local
Response Type:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Twist twist
      geometry_msgs/Vector3 linear
        float64 x : x position
        float64 y : y position
        float64 z : z position
      geometry_msgs/Vector3 angular
        float64 x : linear acceleration along x axis
        float64 y : linear acceleration along y axis
        float64 z : linear acceleration along z axis

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/mavros/local_position/local'

JSON Response:
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

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/mavros/local_position/local',
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
rostopic echo /flytpod/mavros/local_position/local
```

```cpp
#include <core_script_bridge/navigation_bridge.h>

Navigation nav;
nav.position_set(1.0, 3.5, -5.0, 0.12, 5.0, false, false, true, false);
//sends (x,y,z)=(1.0,3.5,-5.0)(m), yaw=0.12rad, tolerance=5.0m, relative=false, async=false, yaw_valid=true, body_frame=false
```

```python
# create flyt_python navigation class instance
from flyt_python import api
drone = api.navigation()
# wait for interface to initialize
time.sleep(3.0)

# Poll data
pos = drone.get_local_position()
# Print the data
print pos.x, pos.vx

```

```cpp--ros
#include <geometry_msgs/TwistStamped>

void lposCallback(const geometry_msgs::TwistStampedConstPtr &lpos)
{
  lpos_data.twist.linear = lpos->twist.linear;
  lpos_data.twist.angular = lpos->twist.angular;
}

ros::NodeHandle nh;
geometry_msgs::TwistStamped lpos_data;
ros::Subscriber sub = nh.subscribe("mavros/local_position/local", 1, lposCallback);
```

```python--ros
from geometry_msgs.msg import TwistStamped

# setup a subscriber and associate a callback function which will be called every time topic is updated.
topic_sub = rospy.Subscriber("/namespace/mavros/local_position/local"), TwistStamped, topic_callback)

# define the callback function which will print the values every time topic is updated
def topic_callback(data):
    x, y, z = data.twist.linear.x, data.twist.linear.y, data.twist.linear.z
    print x, y, z

# unsubscribe from a topic
topic_sub.unregister()  # unregister topic subscription
```

```javascript--REST
$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/mavros/local_position/local",  
    success: function(data){
           console.log(data);
    }
};

```

```javascript--Websocket
var lpos = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/mavros/local_position/local',
    messageType : 'geometry_msgs/TwistStamped',
    throttle_rate: 200
});

var request = new ROSLIB.ServiceRequest({});

lpos.subscribe(request, function(result) {
    console.log(result.twist);
});
```



> Example response

```shell
header: 
  seq: 2589
  stamp: 
    secs: 1489483590
    nsecs: 137668160
  frame_id: fcu
twist: 
  linear: 
    x: 0.0
    y: 0.0
    z: 2.52842187881
  angular: 
    x: 0.000367590633687
    y: 0.001967407763
    z: 0.0995724499226
```

```cpp
instance of geometry_msgs::TwistStamped class
```

```python
instance of class local_position
```

```cpp--ros
instance of geometry_msgs::TwistStamped class
```

```python--ros
instance of gemometry_msgs.msg.TwistStamped class

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
}}

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
}}

```



###Description:

This API subscribes/poles linear position, velocity data in NED frame.  Please check API usage section below before using API.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTFul, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Response:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    x | float | x position in local NED frame.
    y | float | y position in local NED frame.
    z | float | z position in local NED frame.
    vx | float | x velocity in local NED frame.
    vy | float | y velocity in local NED frame.
    vz | float | z velocity in local NED frame.

### ROS endpoint:
All the autopilot state / payload data in FlytOS is shared by ROS topics. Onboard topic subscribers in rospy / roscpp can subscribe to these topics. Take a look at roscpp and rospy API definition for response message structure. 

* Type: Ros Topic</br> 
* Name: /namespace/mavros/local_position/local</br>
* Response Type: geometry_msgs/TwistStamped

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice. All RESTful APIs can poll the data. For telemetry mode (continuous data stream) use websocket APIs.


* URL: ````GET http://<ip>/ros/<namespace>/mavros/local_position/local````
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
}}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/mavros/local_position/local'</br>
* messageType: 'geometry_msgs/TwistStamped'

### API usage information:

* This API provides linear position and lienar velocity.
* Data returned is in NED frame.
* Be careful when using z data obtained into takeoff or position setpoint APIs. These API's may expect z values relative to ground. But the current local position that you get has negative z values for position above ground.

