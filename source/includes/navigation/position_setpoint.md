## Position Setpoint

> Definition

<aside class="warning">
This API is not supported for Fixed Wing.
</aside>

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/position_set
ROS-Service Type: core_api/PositionSet, below is its description

#Request : expects position setpoint via x, y, z
#Request : expects yaw setpoint via yaw (send yaw_valid=true)
geometry_msgs/TwistStamped twist #deprecated, instead use x,y,z,yaw
float32 x
float32 y
float32 z
float32 yaw
float32 tolerance
bool async
bool relative
bool yaw_valid
bool body_frame

#Response : success=true - (if async=false && if setpoint reached before timeout = 30sec) || (if async=true)
bool success
string message
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

Function: position_set(self, x, y, z, yaw=0.0, tolerance=0.0, relative=False, async=False, yaw_valid=False,
                     body_frame=False):
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/position_set
call srv:
    :float x
    :float y
    :float z
    :float yaw
    :float tolerance
    :bool async
    :bool relative
    :bool yaw_valid
    :bool body_frame
response srv: 
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/position_set
call srv:
    :float x
    :float y
    :float z
    :float yaw
    :float tolerance
    :bool async
    :bool relative
    :bool yaw_valid
    :bool body_frame
response srv: 
    :bool success
    :string message
```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/position_set'

JSON Request:
{   
	x: Float,
	y: Float,
	z: Float,
	yaw: Float,
	tolerance: Float,
	async: Boolean,
	relative: Boolean,
	yaw_valid : Boolean,
	body_frame : Boolean
}

JSON Response:
{	
    success: Boolean,
    message: String
}

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/navigation/position_set',
serviceType: 'core_api/PositionSet'

Request:
{   
    x: Float,
    y: Float,
    z: Float,
    yaw: Float,
    tolerance: Float,
    async: Boolean,
    relative: Boolean,
    yaw_valid : Boolean,
    body_frame : Boolean
}

Response:
{   
    success: Boolean,
    message: String
}
```
```python--flyt_python

# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.flyt_python.DroneApiConnector

Function: set_local_position(self,x, y, z, yaw=0.0, tolerance=1.0, async=False, relative=True, yaw_valid=False, body_frame=False)

```

> Example

```shell
rosservice call /flytos/navigation/position_set "{x: 1.0, y: 3.5, z: -5.0, yaw: 0.12, tolerance: 0.0, async: false, relative: false, yaw_valid: true, body_frame: false}"

#sends (x,y,z)=(1.0,3.5,-5.0)(m), yaw=0.12rad, relative=false, async=false, yaw_valid=true, body_frame=false
#default value of tolerance=1.0m if left at 0    
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
nav.position_set(1.0, 3.5, -5.0, 0.12, 2.0, false, false, true, false);
//sends (x,y,z)=(1.0,3.5,-5.0)(m), yaw=0.12rad, tolerance=2.0m, relative=false, async=false, yaw_valid=true, body_frame=false
```

```python
# create flyt_python navigation class instance
from flyt_python import api
drone = api.navigation()
# wait for interface to initialize
time.sleep(3.0)

# command vehicle towards 5 metres SOUTH from current location regardless of heading
drone.position_set(-5, 0, 0, relative=True)
```

```cpp--ros
#include <core_api/PositionSet.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::PositionSet>("/<namespace>/navigation/position_set");
core_api::PositionSet srv;

srv.request.x = 1.0;
srv.request.y = 3.5;
srv.request.z = -5.0;
srv.request.yaw = 0.12;
srv.request.tolerance = 5.0;
srv.request.async = false;
srv.request.yaw_valid = true;
srv.request.relative = false;
srv.request.body_frame = false;
client.call(srv);
bool success = srv.response.success;
std::string message = srv.response.message;

//sends (x,y,z)=(1.0,3.5,-5.0)(m), yaw=0.12rad, tolerance=5.0m, relative=false, async=false, yaw_valid=true, body_frame=false
```

```python--ros
import rospy
from core_api.srv import *

def setpoint_local_position(lx, ly, lz, yaw, tolerance = 2.0, async = False, relative= False, yaw_valid= False, body_frame= False):
    rospy.wait_for_service('/<namespace>/navigation/position_set')
    try:
        handle = rospy.ServiceProxy('/<namespace>/navigation/position_set', PositionSet)
        
        # building message structure
        req_msg = PositionSetRequest(x=lx, y=ly, z=lz, yaw=yaw, tolerance=tolerance, async=async, relative=relative, yaw_valid=yaw_valid, body_frame=body_frame)
        resp = handle(req_msg)
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("pos set service call failed %s", e)

```

```javascript--REST
var  msgdata={};
msgdata["x"]=2.00;
msgdata["y"]=3.00;
msgdata["z"]=-1.00;
msgdata["yaw"]=1.00;
msgdata["tolerance"]=2.00;
msgdata["async"]=true;
msgdata["relative"]=false;
msgdata["yaw_valid"]=true;
msgdata["body_frame"]=false;

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/navigation/position_set",  
    success: function(data){
           console.log(data.success);
           console.log(data.message);
    }
});

```

```javascript--Websocket
var positionSet = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/position_set',
    serviceType : 'core_api/PositionSet'
});

var request = new ROSLIB.ServiceRequest({
    x: 2.00,
    y: 3.00,
    z: -1.00,
    yaw: 1.00,
    tolerance: 2.00,
    async: true,
    relative: false,
    yaw_valid : true,
    body_frame : false
});

positionSet.callService(request, function(result) {
    console.log('Result for service call on '
      + positionSet.name
      + ': '
      + result.success
      +': '
      + result.message);
});
```
```python--flyt_python 
from flyt_python.flyt_python import DroneApiConnector
token = ''                      # Personal Access Token
vehicle_id = ''                 # Vehicle ID

#create an instance of class DroneApiConnector
drone = DroneApiConnector(token,vehicle_id,ip_address='localhost' wait_for_drone_response =True)
drone.connect()
    
drone.set_local_position(x=5, y=0, z=0, yaw=0.0, tolerance=1.0, async=False, relative=True, yaw_valid=False, body_frame=False)
drone.disconnect()
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

```python--flyt_python
{
    success: True, 
    message: message

}
```
###Description:

This API commands the vehicle to go to a specified location in local frame and hover.  Please check API usage section below before using API.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    x, y, z | float | Position Setpoint in NED-Frame (in body-frame if body_frame=true)
    yaw | float | Yaw Setpoint in radians
    yaw_valid | bool | Must be set to true, if yaw 
    tolerance | float | Acceptance radius in meters, default value=1.0m 
    relative | bool | If true, position setpoints relative to current position is sent
    async | bool | If true, asynchronous mode is set
    body_frame | bool | If true, position setpoints are relative with respect to body frame
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### API usage information:

* Vehicle should be in GUIDED or OFFBOARD or API|POSCTL mode for this API to work.
* Vehicle should be armed for this API to work.
* Do not call this API when vehicle is grounded. Use take_off API first to get the vehicle in air.
* X,Y,Z are position setpoints in 3 linear axes. Yaw is angular rotation around Z axis. Right hand notation is used to find positive yaw direction.
* Effect of parameters:
  * Async:
     * True: The API call would return as soon as the command has been sent to the autopilot, irrespective of whether the vehicle has reached the given setpoint or not.
     * False: The API call would wait for the function to return, which happens when either the position setpoint is reached or timeout=30secs is over.
  * Relative: 
     * True: Linear position setpoints (x,y,z) are calculated from current location. Home location is not relevant. Yaw is calculated from North.
     * False: Linear position setpoints (x,y,z) are calculated from home location. Home location is reset every time vehicle arms. Yaw is calculated from North. 
  * Body_frame 
     * True: All the setpoints are converted to body frame. 
        * Front of vehicle : +x
        * Right of vehicle : +y
        * Down: +z
        * Yaw is calculated from front of vehicle. 
     * False: All the setpoints are converted to local NED (North, East, Down) frame. Yaw is calculated from North. 
* Either body_frame or relative flag can be set to true at a time. If both are set then only body_frame is effective.
* For yaw setpoint to be effective the yaw_valid argument must be set to true.
* This API overrides any previous mission / navigation API being carried out.
* This API requires position lock. GPS, Optical Flow, VICON system can provide position data to vehicle.
* To provide only Yaw setpoint use this API with x,y,z arguments set to 0, relative=True, yaw_valid=True
* Following parameters need to be manually configured according to vehicle frame.
  * MPC_XY_VEL_MAX : Maximum horizontal velocity. For smaller and lighter this parameter could be set to value between 8 m/s to 15 m/s. For larger and heavier systems it is safer to set this value below 8 m/s.
  * MPC_Z_VEL_MAX : Maximum vertical velocity. For smaller and lighter this parameter could be set to value between 3 m/s to 10 m/s. For larger and heavier systems it is safer to set this value below 8 m/s.
  * Vehicle will try to go to the setpoint with maximum velocity. At no point the current velocity will exceed limit set by above parameters. So if you want the vehicle to reach a point slowly then reducen the value of above paramters.

### ROS endpoint:

Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: `Ros Service`
* Name: `/<namespace>/navigation/position_set`
* Service Type: `core_api/PositionSet`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port **80**. RESTful APIs can be called from remote platform of your choice.

* URL: `POST http://<ip>/ros/<namespace>/navigation/position_set`
* JSON Request:
`{
    x: Float,
    y: Float,
    z: Float,
    yaw: Float,
    tolerance: Float,
    async: Boolean,
    relative: Boolean,
    yaw_valid : Boolean,
    body_frame : Boolean
}`
* JSON Response:
`{
    success: Boolean
    message: String
}`


### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 

Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace>/navigation/position_set`
* serviceType: `core_api/PositionSet`
