## Global Position Setpoint


> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/position_set_global
ROS-Service Type: core_api/PositionSetGlobal, below is its description

#Request : expects position setpoint via lat_x, long_y, rel_alt_z(altitude from home)
#Request : expects yaw setpoint via yaw (send yaw_valid=true)
geometry_msgs/TwistStamped twist #deprecated, instead use lat_x,long_y,rel_alt_z,yaw
float32 lat_x
float32 long_y
float32 rel_alt_z
float32 yaw
float32 tolerance
bool async
bool yaw_valid

#Response : return success=true, (if async=false && if setpoint reached before timeout = 30sec) || (if async=true && command sent to autopilot)
bool success
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition:     int position_set_global(float lat_x, float long_y, float rel_alt_z, float yaw=0, float tolerance=0, bool async=false, bool yaw_valid=false);
Arguments:
    :lat_x: Latitude
    :long_y: Longitude
    :rel_alt_z: Altitue (Positive distance upwards from home position)
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
Name: /<namespace>/navigation/position_set_global
call srv:
    :float lat_x
    :float long_y
    :float rel_alt_z
    :float yaw
    :float tolerance
    :bool async
    :bool yaw_valid

response srv: 
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/position_set_global
call srv:
    :float lat_x
    :float long_y
    :float rel_alt_z
    :float yaw
    :float tolerance
    :bool async
    :bool yaw_valid

```

```javascript--REST
This is a REST call for the API. Make sure to replace
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/position_set_global'

JSON Request:
{   
    lat_x: Float,
    long_y: Float,
    rel_alt_z: Float,
    yaw: Float,
    tolerance: Float,
    async: Boolean,
    relative: Boolean,
    yaw_valid : Boolean,
    body_frame : Boolean }

JSON Response:
{   success: Boolean,
    message: String, }

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you
initialise the websocket using websocket initialising
API and replace namespace with the namespace of
the FlytOS running device before calling the API
with websocket.

name: '/<namespace>/navigation/position_set_global',
serviceType: 'core_api/PositionSetGlobal'

Request:
{       
    lat_x: Float,
    long_y: Float,
    rel_alt_z: Float,
    yaw: Float,
    tolerance: Float,
    async: Boolean,
    relative: Boolean,
    yaw_valid : Boolean }

Response:
{   success: Boolean,
    message: String, }


```


> Example

```shell

rosservice call /flytpod/navigation/position_set_global "{ lat_x: 8.04303, long_y: 43.57437, rel_alt_z: 5.0, yaw: 0.12 ,tolerance: 0.0, async: false, yaw_valid: true}"

#sends (Lat,Lon,relAlt)=(8.04303, 43.57437,5.0)(m), yaw=0.12rad, async=false, yaw_valid=true
#default value of tolerance=1.0m if left at 0    
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
nav.position_set_global(10.342124, 13.4323233, 5.0, 0.12, 2.0, false, true);
//sends (Lat,Lon,relAlt)=(10.342124, 13.4323233, 5.0)(m), yaw=0.12rad, tolerance=2.0m, async=false, yaw_valid=true
```

```python
# create flyt_python navigation class instance
from flyt_python import api
drone = api.navigation()
# wait for interface to initialize
time.sleep(3.0)

# send vehicle to GPS coordinate with height 10 meters above home position.
drone.position_set_global(10.342124, 13.4323233, 10)

```

```cpp--ros
#include <core_api/PositionSetGlobal.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::PositionSetGlobal>("/<namespace>/navigation/position_set_global");
core_api::PositionSetGlobal srv;

srv.lat_x = 10.342124;
srv.long_y = 13.4323233;
srv.rel_alt_z = 5.0;
srv.yaw = 0.5;
srv.request.tolerance = 2.0;
srv.request.async = true;
srv.request.yaw_valid = true;
client.call(srv);
success = srv.response.success;
```

```python--ros
import rospy
from core_api.srv import *

def setpoint_global_position(lat_x, long_y, rel_alt_z, yaw, tolerance= 0.0, async = False, yaw_valid= False):
    rospy.wait_for_service('/<namespace>/navigation/position_set_global')
    try:
        handle = rospy.ServiceProxy('/<namespace>/navigation/position_set_global', PositionSetGlobal)

        # build message structure
        req_msg = PositionSetGlobalRequest(lat_x=lat_x, long_y=long_y, rel_alt_z=rel_alt_z, yaw=yaw, tolerance=tolerance, async=async, yaw_valid=yaw_valid)
        resp = handle(req_msg)
        return resp

    except rospy.ServiceException, e:
        rospy.logerr("global pos set service call failed %s", e)
```

```javascript--REST
var  msgdata={};
msgdata["lat_x"]=10.342124;
msgdata["long_y"]=13.4323233;
msgdata["rel_alt_z"]=5.00;
msgdata["yaw"]=1.00;
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
    lat_x: 10.342124,
    long_y: 13.4323233,
    rel_alt_z: 5.00,
    yaw: 1.00,
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
    lat_x | float | Latitude
    long_y | float | Longitude
    rel_alt_z | float | relative height from current location in meters
    yaw | float | Yaw Setpoint in radians
    yaw_valid | bool | Must be set to true, if yaw
    tolerance | float | Acceptance radius in meters, default value=1.0m
    async | bool | If true, asynchronous mode is set

    Output:

    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### ROS endpoint:
Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure.

* Type: Ros Service</br>
* Name: /\<namespace\>/navigation/position_set_global</br>
* Service Type: PositionSetGlobal

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ``POST http://<ip>/ros/<namespace>/navigation/position_set_global``
* JSON Request:
{
    lat_x: Float,
    long_y: Float,
    rel_alt_z: Float,
    yaw: Float,
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

* Vehicle should be in GUIDED or OFFBOARD or API|POSCTL mode for this API to work.
* Vehicle should be armed for this API to work.
* Do not call this API when vehicle is grounded. Use take_off API first to get the vehicle in air.
* Right hand notation is used to find positive yaw direction.
* rel_alt_z parameter is always positive.
* rel_alt_z parameter should be calculated relative to ground. E.g. If vehicle is at position A hovering above ground at 10 meters and is then commanded to reach to point B which is 5 meters higher than point A then rel_alt_z should be 10+5=15.
* Effect of parameters:
  * Async:
     * True: The API call would return as soon as the command has been sent to the autopilot, irrespective of whether the vehicle has reached the given setpoint or not.
     * False: The API call would wait for the function to return, which happens when either the position setpoint is reached or timeout=30secs is over.
* For yaw setpoint to be effective the yaw_valid argument must be set to true.
* This API overrides any previous mission / navigation API being carried out.
* This API requires global position lock. So using a GPS receiver is must for this API to work.
* Following parameters need to be manually configured according to vehicle frame.
  * MPC_XY_VEL_MAX : Maximum horizontal velocity. For smaller and lighter this parameter could be set to value between 8 m/s to 15 m/s. For larger and heavier systems it is safer to set this value below 8 m/s.
  * MPC_Z_VEL_MAX : Maximum vertical velocity. For smaller and lighter this parameter could be set to value between 3 m/s to 10 m/s. For larger and heavier systems it is safer to set this value below 8 m/s.
  * Vehicle will try to go to the setpoint with maximum velocity. At no point the current velocity will exceed limit set by above parameters. So if you want the vehicle to reach a point slowly then reducen the value of above paramters.
