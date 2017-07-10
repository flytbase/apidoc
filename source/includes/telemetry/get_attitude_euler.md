## Get Attitude Euler Data



> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Topic Name: /<namespace>/mavros/imu/data_euler
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

Function Definition: sysSubscribe(Navigation::vehicle_attitude_euler,attitudeEulerCb);

Arguments:
    vehicle_attitude_euler: This argument selects vehicle attitude euler topic to be subscribed
    attitudeEulerCb: Callback function for the subscribed attitude messages

Returns: Vehicle attitude in euler notation in ros geometry_msgs::TwistStamped message structure
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

Function: get_attitude_euler()

Response: attitude_euler_object as described below.
    class attitude_euler:
        '''
        Holds fields for Attitude data in Euler Angles
        '''
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        rollspeed = 0.0
        pitchspeed = 0.0
        yawspeed = 0.0

This API support single poll mode only.
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Topic
Name: /<namespace>/mavros/imu/data_euler
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
Name: /<namespace>/mavros/imu/data_euler
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
        z: Float},
    angular:{
        x: Float,
        y: Float,
        z: Float}
}}
```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/mavros/imu/data_euler',
messageType: 'geometry_msgs/TwistStamped'

Response:
{   twist:{
    linear:{
        x: Float,
        y: Float,
        z: Float},
    angular:{
        x: Float,
        y: Float,
        z: Float}
}}

```


> Example

```shell
rostopic echo /flytpods/mavros/imu/data_euler 
```

```cpp
#include <cpp_api.navigation_bridge.h>

Navigation nav;
geometry_msgs::TwistStamped att_euler;

void attitudeEulerCb(void *_att_euler)
{
    att_euler = * (geometry_msgs::TwistStamped*)(_att_euler);
    std::cout<<"\nroll \t\tpitch \t\tyaw \t\trollspeed \tpitchspeed \tyawspeed";
    std::cout<<"\n"<<att_euler.twist.linear.x<<"\t"<<att_euler.twist.linear.y<<"\t"<<att_euler.twist.linear.z;
    std::cout<<"\t"<<att_euler.twist.angular.x<<"\t"<<att_euler.twist.angular.y<<"\t"<<att_euler.twist.angular.z;
    fflush(stdout);
}

int main(int argc, char *argv[])
{
	nav.sysSubscribe(Navigation::vehicle_attitude_euler,attitudeEulerCb);
  while(1){
		if(!ros::ok())
		  exit(0);
		sleep(0.1);
  }
}
```

```python
# create flyt_python navigation class instance
from flyt_python import api
drone = api.navigation()
# wait for interface to initialize
time.sleep(3.0)

# Poll attitude euler data
att = drone.get_attitude_euler()
# Print the data
print att.roll, att.pitch, att.yaw, att.rollspeed, att.pitchspeed, att.yawspeed

```

```cpp--ros
#include <geometry_msgs/TwistStamped.h>

void attCallback(const geometry_msgs::TwistStampedConstPtr &att)
{
  std::cout<<"\nroll \t\tpitch \t\tyaw \t\trollspeed \tpitchspeed \tyawspeed";
  std::cout<<"\n"<<att->twist.linear.x<<"\t"<<att->twist.linear.y<<"\t"<<att->twist.linear.z;
  std::cout<<"\t"<<att->twist.angular.x<<"\t"<<att->twist.angular.y<<"\t"<<att->twist.angular.z;
  fflush(stdout);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "att_euler_cb");
		ros::NodeHandle nh;
		ros::Subscriber sub = nh.subscribe("/<namespace>/mavros/imu/data_euler", 1, attCallback);
		ros::spin();
		return 0;
}
```

```python--ros
import rospy
from geometry_msgs.msg import TwistStamped

# setup a subscriber and associate a callback function which will be called every time topic is updated.
topic_sub = rospy.Subscriber("/<namespace>/mavros/imu/data_euler"), TwistStamped, topic_callback)

# define the callback function which will print the values every time topic is updated
def topic_callback(data):
    roll, pitch, yaw = data.twist.linear.x, data.twist.linear.y, data.twist.linear.z
    print roll, pitch, yaw

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
var imuEulerData = new ROSLIB.Topic({
    ros : ros,
    name : '/<namespace>/mavros/imu/data_euler',
    messageType : 'geometry_msgs/TwistStamped'
});

imuEulerData.subscribe(function(message) {
    console.log(message.data);
});
```


> Example response

```shell
header: 
  seq: 4090
  stamp: 
    secs: 1489483667
    nsecs: 763130240
  frame_id: fcu
twist: 
  linear: 
    x: -0.0357596799731
    y: -0.0206729210913
    z: -1.89611303806
  angular: 
    x: -0.00441705761477
    y: 0.00617094011977
    z: -0.000732765707653
```

```cpp
instance of geometry_msgs::TwistStamped class

header: 
  seq: 2041
  stamp: 
    secs: 1492705227
    nsecs: 960368337
  frame_id: fcu
twist: 
  linear: 
    x: -0.00561133073643
    y: -0.00531742209569
    z: -0.0351081602275
  angular: 
    x: 0.00158157129772
    y: 0.001551638823
    z: 0.00154603447299
```

```python
instance of class
class attitude_euler:
    '''
    Holds fields for Attitude data in Euler Angles
    '''
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    rollspeed = 0.0
    pitchspeed = 0.0
    yawspeed = 0.0

```

```cpp--ros
instance of geometry_msgs::TwistStamped class

header: 
  seq: 2041
  stamp: 
    secs: 1492705227
    nsecs: 960368337
  frame_id: fcu
twist: 
  linear: 
    x: -0.00561133073643
    y: -0.00531742209569
    z: -0.0351081602275
  angular: 
    x: 0.00158157129772
    y: 0.001551638823
    z: 0.00154603447299
```

```python--ros
instance of gemometry_msgs.msg.TwistStamped class

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

```javascript--REST
{
    twist:{
    linear:{
        x: Float,
        y: Float,
        z: Float},
    angular:{
        x: Float,
        y: Float,
        z: Float}
}
```

```javascript--Websocket
{
    twist:{
    linear:{
        x: Float,
        y: Float,
        z: Float},
    angular:{
        x: Float,
        y: Float,
        z: Float}
}
```



###Description:

This API subscribes/polls attitude data (angle and angular rate) in euler angles.  Please check API usage section below before using API.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Response:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    roll | float | roll angle in radians, NED frame.
    pitch | float | pitch angle in radians, NED frame.
    yaw | float | yaw angle in radians, NED frame.
    rollspeed | float | roll rate in radians/sec, NED frame.
    pitchspeed | float | pitch rate in radians/sec, NED frame.
    yawspeed | float | yaw rate in radians/sec, NED frame.

### ROS endpoint:
All the autopilot state / payload data in FlytOS is shared by ROS topics. Onboard topic subscribers in rospy / roscpp can subscribe to these topics. Take a look at roscpp and rospy API definition for response message structure. 

* Type: Ros Topic</br> 
* Name: /\<namespace\>/mavros/imu/data_euler</br>
* Response Type: geometry_msgs/TwistStamped

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice. All RESTful APIs can poll the data. For telemetry mode (continuous data stream) use websocket APIs.

* URL: ``GET http://<ip>/ros/<namespace>/mavros/imu/data_euler``
* JSON Response:
{
    twist:{
    linear:{
        x: Float,
        y: Float,
        z: Float},
    angular:{
        x: Float,
        y: Float,
        z: Float}
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/\<namespace\>/mavros/imu/data_euler'</br>
* messageType: 'geometry_msgs/TwistStamped'

### API usage information:

* This API provides roll, pitch, yaw, rollspeed, pitchspeed, yawspeed information.
* Data returned is in NED frame.
