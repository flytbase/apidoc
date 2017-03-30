## Global Position Setpoint


> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/position_set_global
ROS-Service Type: core_api/PositionSetGlobal, below is its description

#Request : expects position setpoint via twist.twist.linear.x,linear.y,linear.z which corresponds respectively to the desired Latitude, longitude and altitude 
#Request : expects yaw setpoint via twist.twist.angular.z (send yaw_valid=true)
geometry_msgs/TwistStamped twist
float32 tolerance
bool async
bool yaw_valid

#Response : return success=true, (if async=false && if setpoint reached before timeout = 30sec) || (if async=true && command sent to autopilot)
bool success
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition:     int position_set_global(float lat, float lon, float alt, float yaw=0, float tolerance=0, bool async=false, bool yaw_valid=false);
Arguments:
    :lat: Latitude
    :lon: Longitude
    :alt: Altitue (Positive distance upwards from home position)
    :yaw: Yaw Setpoint in radians
    :yaw_valid: Must be set to true, if yaw setpoint is provided
    :tolerance: Acceptance radius in meters, default value=1.0m
    :async: If true, asynchronous mode is set
    :returns: For async=true, returns 0 if the command is successfully sent to the vehicle, else returns 1. For async=false, returns 0 if the vehicle reaches given setpoint before timeout=30secs, else returns 1.
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.api.navigation

Function: position_set_global(self, lat, lon, rel_ht, yaw=0.0, tolerance=0.0, async=False, yaw_valid=False):
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/position_set_global()
call srv:
    :geometry_msgs/TwistStamped twist
    :float32 tolerance
    :bool async
    :bool yaw_valid

response srv: bool success
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/position_set_global()
call srv:
    :geometry_msgs/TwistStamped twist
    :float32 tolerance
    :bool async
    :bool yaw_valid

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/position_set_global'

JSON Request:
{   twist:{twist:{  linear:{
                x: Float,
                y: Float,
                z: Float
            },angular:{
                z: Float
    }}},
    tolerance: Float,
    async: Boolean,
    relative: Boolean,
    yaw_valid : Boolean,
    body_frame : Boolean }

JSON Response:
{   success: Boolean, }

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/navigation/position_set_global',
serviceType: 'core_api/PositionSetGlobal'

Request:
{   twist:{twist:{  linear:{
                x: Float,
                y: Float,
                z: Float
            },angular:{
                z: Float
    }}},
    tolerance: Float,
    async: Boolean,
    relative: Boolean,
    yaw_valid : Boolean }

Response:
{   success: Boolean, }


```


> Example

```shell

rosservice call /flytpod/navigation/position_set_global "{twist: {header: {seq: 0,stamp: {secs: 0, nsecs: 0}, frame_id: ''},twist: {linear: {x: 18.5204303, y:  73.8567437, z: -5.0}, angular: {x: 0.0, y: 0.0, z: 0.12}}}, tolerance: 0.0, async: false, yaw_valid: true}"

#sends (Lat,Lon,Alt)=(18.5204303, 73.8567437,5.0)(m), yaw=0.12rad, async=false, yaw_valid=true
#default value of tolerance=1.0m if left at 0    
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
nav.position_set_global(18.7342124, 73.4323233, 5.0, 0.12, 2.0, false, false, true, false);
#sends (x,y,z)=(1.0,3.5,5.0)(m), yaw=0.12rad, tolerance=2.0m, relative=false, async=false, yaw_valid=true, body_frame=false
```

```python
# create flyt_python navigation class instance
from flyt_python import api
drone = api.navigation()
# wait for interface to initialize
time.sleep(3.0)

# send vehicle to GPS coordinate with height 10 meters above current height.
drone.position_set_global(18.7342124, 73.4323233, 10)

```

```cpp--ros
#include <core_api/PositionSetGlobal.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::PositionSetGlobal>("navigation/position_set_global");
core_api::PositionSetGlobal srv;

srv.request.twist.twist.angular.z = 0.5;
srv.request.twist.twist.linear.x = 4,0;
srv.request.twist.twist.linear.y = 3.0;
srv.request.twist.twist.linear.z = 5.0;
srv.request.tolerance = 2.0;
srv.request.async = true;
srv.request.yaw_valid = true;
client.call(srv);
success = srv.response.success;
```

```python--ros
from core_api.srv import * 

def setpoint_global_position(lat, lon, alt, yaw, tolerance= 0.0, async = False, yaw_valid= False):
    rospy.wait_for_service(/<namespace>/navigation/position_set_global')
    try:
        handle = rospy.ServiceProxy(/<namespace>/navigation/position_set_global', PositionSetGlobal)
        
        # build message structure
        header_msg = std_msgs.msg.Header(1,rospy.Time(0.0,0.0),'a')
        twist = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(lat,lon,alt),geometry_msgs.msg.Vector3(0.0,0.0,yaw))
        twiststamped_msg= geometry_msgs.msg.TwistStamped(header_msg, twist)
        req_msg = VelocitySetRequest(twiststamped_msg, tolerance, async, yaw_valid)
        resp = handle(req_msg)
        return resp
    
    except rospy.ServiceException, e:
        rospy.logerr("global pos set service call failed %s", e)

```

```javascript--REST
var  msgdata={};
msgdata["twist"]={};
msgdata.twist["twist"]={};
masdata.twist.twist["linear"]={};
msgdata.twist.twist.linear["x"]=18.594061;
msgdata.twist.twist.linear["y"]=73.911037;
msgdata.twist.twist.linear["z"]=-1.00;
msgdata.twist.twist["angular"]={};
msgdata.twist.twist.angular["z"]=1.00;
msgdata["tolerance"]=2.00;
msgdata["async"]=true;
msgdata["relative"]=false;
msgdata["yaw_valid"]=true;

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/navigation/position_set_global",  
    success: function(data){
           console.log(data.success);
    }
};

```

```javascript--Websocket
var positionSetGlobal = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/position_set_global',
    serviceType : 'core_api/PositionSetGlobal'
});

var request = new ROSLIB.ServiceRequest({
    twist:{twist:{  linear:{
                x: 18.594061,
                y: 73.911037,
                z: -1.00
            },angular:{
                z: 1.00
    }}},
    tolerance: 2.00,
    async: true,
    relative: false,
    yaw_valid : true
});

positionSetGlobal.callService(request, function(result) {
    console.log('Result for service call on '
      + positionSetGlobal.name
      + ': '
      + result.success);
});
```


> Example response

```shell
success: true
```

```cpp
0
```

```python
True
```

```cpp--ros
success: True
```

```python--ros
Success: True
```

```javascript--REST
{
    success:True
}

```

```javascript--Websocket
{
    success:True
}
```





###Description:

This API sets a desired position setpoint in global coordinate system (WGS84). Please check API usage section below before using API. 

###Parameters:
    
    Following parameters are applicable for onboard C++ and Python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    lat | float | Latitude
    lon | float | Longitude
    rel_ht | float | relative height from current location in meters
    yaw | float | Yaw Setpoint in radians
    yaw_valid | bool | Must be set to true, if yaw 
    tolerance | float | Acceptance radius in meters, default value=1.0m 
    async | bool | If true, asynchronous mode is set
    
    Output:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful

### ROS endpoint:
Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy api definition for message structure. 

* Type: Ros Service</br> 
* Name: /\<namespace\>/navigation/position_set_global</br>
* Service Type: PositionSetGlobal

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ````POST http://<ip>/ros/<namespace>/navigation/position_set_global````
* JSON Request:
{
    twist:{
        twist:{
            linear:{
                x: Float,
                y: Float,
                z: Float
            },
            angular:{
                z: Float
            }
        }
    },
    tolerance: Float,
    async: Boolean,
    relative: Boolean,
    yaw_valid : Boolean
}
* JSON Response:
{
    success: Boolean
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/\<namespace\>/navigation/position_set_global'</br>
* serviceType: 'core_api/PositionSetGlobal'


### API usage information:

* Vehicle should be in OFFBOARD/API_CTL mode for this API to work.
* Vehicle should be armed for this API to work.
* Do not call this API when vehicle is grounded. Use take_off API first to get the vehicle in air.
* Right hand notation is used to find positive yaw direction.
* rel_ht parameter is always positive.
* rel_ht parameter should be calculated relative to ground. E.g. If vehicle is at position A hovering above ground at 10 meters and is then commanded to reach to point B which is 5 meters higher than point A then rel_ht should be 10+5=15. 
* Effect of parameters:
  * Async:
     * True: The API call would return as soon as the command has been sent to the autopilot, irrespective of whether the vehicle has reached the given setpoint or not.
     * False: The API call would wait for the function to return, which happens when either the position setpoint is reached or timeout=30secs is over. 
* For yaw setpoint to be effective the yaw_valid argument must be set to true.
* This API overrides any previous mission / navigation API being carried out.
* This API requires global position lock. So using a GPS receiver is must for this API to work.
* * Following parameters need to be manually configured according to vehicle frame.
  * MPC_XY_VEL_MAX : Maximum horizontal velocity. For smaller and lighter this parameter could be set to value between 8 m/s to 15 m/s. For larger and heavier systems it is safer to set this value below 8 m/s.
  * MPC_Z_VEL_MAX : Maximum vertical velocity. For smaller and lighter this parameter could be set to value between 3 m/s to 10 m/s. For larger and heavier systems it is safer to set this value below 8 m/s.
  * Vehicle will try to go to the setpoint with maximum velocity. At no point the current velocity will exceed limit set by above parameters. So if you want the vehicle to reach a point slowly then reducen the value of above paramters.

