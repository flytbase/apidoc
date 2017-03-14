# Telemetry APIs

## Get Attitude Quaternion



> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

ROS-Topic Name: /<namespace>/mavros/imu/data
ROS-Topic Type: sensor_msgs/Imu, below is its description

#Subscriber response : Attitude Quaternion 
Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
    float64[9] orientation_covariance
    geometry_msgs/Vector3 angular_velocity
      float64 x
      float64 y
      float64 z
    float64[9] angular_velocity_covariance
    geometry_msgs/Vector3 linear_acceleration
      float64 x
      float64 y
      float64 z
    float64[9] linear_acceleration_covariance

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

Function: get_attitude_quaternion()

Response: attitude_quaternion as described below.
    class attitude_quaternion:
        '''
        Holds fields for Attitude data in Quaternion format
        '''
        x = 0.0
        y = 0.0
        z = 0.0
        w = 0.0
        rollspeed = 0.0
        pitchspeed = 0.0
        yawspeed = 0.0

This API supports single pole mode only.
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Topic
Name: /<namespace>/mavros/imu/data
Response Type: sensor_msgs/Imu
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
    float64[9] orientation_covariance
    geometry_msgs/Vector3 angular_velocity
      float64 x
      float64 y
      float64 z
    float64[9] angular_velocity_covariance
    geometry_msgs/Vector3 linear_acceleration
      float64 x
      float64 y
      float64 z
    float64[9] linear_acceleration_covariance

```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Topic
Name: /<namespace>/mavros/imu/data
Response Type: sensor_msgs/Imu
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
    float64[9] orientation_covariance
    geometry_msgs/Vector3 angular_velocity
      float64 x
      float64 y
      float64 z
    float64[9] angular_velocity_covariance
    geometry_msgs/Vector3 linear_acceleration
      float64 x
      float64 y
      float64 z
    float64[9] linear_acceleration_covariance

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.
    
URL: 'http://<ip>/ros/<namespace>/mavros/imu/data'

JSON Response:
{   orientation:{
        x: Float,
        y: Float,
        z: Float},
    angular_velocity:{
        x: Float,
        y: Float,
        z: Float},
    linear_acceleration:{
        x: Float,
        y: Float,
        z: Float}
}

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/mavros/imu/data',
messageType: 'sensor_msgs/Imu'

Response:
{   orientation:{
        x: Float,
        y: Float,
        z: Float},
    angular_velocity:{
        x: Float,
        y: Float,
        z: Float},
    linear_acceleration:{
        x: Float,
        y: Float,
        z: Float}
}
```


> Example

```shell
rostopic echo /flytpod/mavros/imu/data
```

```cpp


```

```python
# create flyt_python navigation class instance
from flyt_python import api
drone = api.navigation()
# wait for interface to initialize
time.sleep(3.0)

# Poll attitude euler data
att = drone.get_attitude_quaternion()
# Print the data
print att.x, att.y, att.z, att.w, att.rollspeed, att.pitchspeed, att.yawspeed

```

```cpp--ros
#include <sensor_msgs/Imu.h>


void attCallback(const sensor_msgs::ImuConstPtr &att)
{
  att_data.orientation = att->orientation;
  att_data.angular_velocity = att->angular_velocity;
  att_data.linear_acceleration = att->linear_acceleration;
}

ros::NodeHandle nh;
sensor_msgs::Imu att_data;
ros::Subscriber sub = nh.subscribe("mavros/imu/data", 1, attCallback);
```

```python--ros
from sensor_msgs.msg import Imu

# setup a subscriber and associate a callback function which will be called every time topic is updated.
topic_sub = rospy.Subscriber("/namespace/mavros/imu/data"), Imu, topic_callback)

# define the callback function which will print the values every time topic is updated
def topic_callback(data):
    x, y, z, w= data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
    print x, y, z, w

# unsubscribe from a topic
topic_sub.unregister()  # unregister topic subscription
```

```javascript--REST

$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/mavros/imu/data",  
    success: function(data){
           console.log(data);
    }
};

```

```javascript--Websocket
var imuData = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/mavros/imu/data',
    messageType : 'sensor_msgs/Imu',
    throttle_rate: 200
});

var request = new ROSLIB.ServiceRequest({});

imuData.subscribe(request, function(result) {
    console.log(result.data);
});
```



> Example response

```shell
header: 
  seq: 112
  stamp: 
    secs: 1489476690
    nsecs: 278339713
  frame_id: fcu
orientation: 
  x: -0.00593215392702
  y: 0.00396701722143
  z: 0.988477372188
  w: 0.151200386894
orientation_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
angular_velocity: 
  x: 0.00392133416608
  y: -0.000496329041198
  z: -0.000130902582896
angular_velocity_covariance: [1.2184696791468346e-07, 0.0, 0.0, 0.0, 1.2184696791468346e-07, 0.0, 0.0, 0.0, 1.2184696791468346e-07]
linear_acceleration: 
  x: -0.1176798
  y: -0.4314926
  z: -9.81645665
linear_acceleration_covariance: [8.999999999999999e-08, 0.0, 0.0, 0.0, 8.999999999999999e-08, 0.0, 0.0, 0.0, 8.999999999999999e-08]
```

```cpp
instance of sensor_msgs::Imu class
```

```python
instance of class attitude_quaternion
```

```cpp--ros
instance of sensor_msgs::Imu class
```

```python--ros
instance of sensor_msgs.msg.Imu class

```

```javascript--REST
{
    orientation:{
        x: Float,
        y: Float,
        z: Float},
    angular_velocity:{
        x: Float,
        y: Float,
        z: Float},
    linear_acceleration:{
        x: Float,
        y: Float,
        z: Float}
}

```

```javascript--Websocket
{
    orientation:{
        x: Float,
        y: Float,
        z: Float},
    angular_velocity:{
        x: Float,
        y: Float,
        z: Float},
    linear_acceleration:{
        x: Float,
        y: Float,
        z: Float}
}

```




###Description:

This API subscribes/poles attitude data (angle and angular rate) in quaternion.  Please check API usage section below before using API.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTFul, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Response:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    x | float | x vector.
    y | float | y vector.
    z | float | z vector.
    w | float | w vector.
    rollspeed | float | roll rate in radians/sec, NED frame.
    pitchspeed | float | pitch rate in radians/sec, NED frame.
    yawspeed | float | yaw rate in radians/sec, NED frame.

### ROS endpoint:
All the autopilot state / payload data in FlytOS is shared by ROS topics. Onboard topic subscribers in rospy / roscpp can subscribe to these topics. Take a look at roscpp and rospy API definition for response message structure. 

* Type: Ros Topic</br> 
* Name: /namespace/mavros/imu/data</br>
* Response Type: sensor_msgs/Imu

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice. All RESTful APIs can poll the data. For telemetry mode (continuous data stream) use websocket APIs.


* URL: ````GET http://<ip>/ros/<namespace>/mavros/imu/data````
* JSON Response:
{
    orientation:{
        x: Float,
        y: Float,
        z: Float},
    angular_velocity:{
        x: Float,
        y: Float,
        z: Float},
    linear_acceleration:{
        x: Float,
        y: Float,
        z: Float}
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/mavros/imu/data'</br>
* messageType: 'sensor_msgs/Imu'

### API usage information:

* This API provides orientation in quaternion and angular velocity
