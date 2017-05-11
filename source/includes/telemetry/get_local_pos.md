## Get Local Position 


> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

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

Function Definition: sysSubscribe(Navigation::local_position,lposCb);

Arguments:
    local_position: This argument selects local position topic to be subscribed
    lposCb: Callback function for the subscribed local position messages

Returns: local position in ros geometry_msgs::TwistStamped message structure
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
    
This API support single poll mode only.
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
API and replace namespace with the namespace of 
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
#include <cpp_api/navigation_bridge.h>

Navigation nav;
geometry_msgs::TwistStamped lpos;

void lposCb(void *_lpos)
{
    lpos = * (geometry_msgs::TwistStamped*)(_lpos);
}
nav.sysSubscribe(Navigation::local_position,lposCb);

std::cout << lpos << std::endl;
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
ros::Subscriber sub = nh.subscribe("/<namespace>/mavros/local_position/local", 1, lposCallback);
```

```python--ros
from geometry_msgs.msg import TwistStamped

# setup a subscriber and associate a callback function which will be called every time topic is updated.
topic_sub = rospy.Subscriber("/<namespace>/mavros/local_position/local"), TwistStamped, topic_callback)

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

This API subscribes/polls linear position, velocity data in NED frame.  Please check API usage section below before using API.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTFul, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Response:
    
    Parameter | Type | Description
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
* Name: /\<namespace\>/mavros/local_position/local</br>
* Response Type: geometry_msgs/TwistStamped

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice. All RESTful APIs can poll the data. For telemetry mode (continuous data stream) use websocket APIs.


* URL: ``GET http://<ip>/ros/<namespace>/mavros/local_position/local``
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

* name: '/\<namespace\>/mavros/local_position/local'</br>
* messageType: 'geometry_msgs/TwistStamped'

### API usage information:

* This API provides linear position and lienar velocity.
* Data returned is in NED frame.
* Be careful when using z data obtained into takeoff or position setpoint APIs. These API's may expect z values relative to ground. But the current local position that you get has negative z values for position above ground.

