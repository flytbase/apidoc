## Velocity Setpoint

> Definition

<aside class="warning">
This API is not supported for Fixed Wing.
</aside>


```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/velocity
ROS-Service Type: core_api/VelocitySet, below is its description

#Request : expects velocity setpoint via vx,vy,vz
#Request : expects yaw_rate setpoint via yaw_rate (send yaw_rate_valid=true)
geometry_msgs/TwistStamped twist #deprecated, instead use vx,vy,vz,yaw_rate
float32 vx
float32 vy
float32 vz
float32 yaw_rate
float32 tolerance
bool async
bool relative
bool yaw_rate_valid
bool body_frame

#Response : return success=true, (if async=false && if setpoint reached before timeout = 30sec) || (if async=true && command sent to autopilot)
bool success
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition:  int Navigation::velocity_set(float vx, float vy, float vz, float yaw_rate = 0, float tolerance = 0, bool relative = false, bool async = false, bool yaw_rate_valid = false, bool body_frame = false)

Arguments:
    vx,vy,vz: Velocity Setpoint in NED-Frame (in body-frame if body_frame=true)
    yaw_rate: Yaw_rate Setpoint in radians/sec
    yaw_rate_valid: Must be set to true, if yaw_rate setpoint is provided
    tolerance: Acceptance radius in meters/s, default value=1.0m/s
    relative: If true, velocity setpoints relative to current position is sent
    async: If true, asynchronous mode is set
    body_frame: If true, velocity setpoints are with respect to body frame

Returns: For async=true, returns 0 if the command is successfully sent to the vehicle, else returns 1. For async=false, returns 0 if the vehicle reaches given setpoint before timeout=30secs, else returns 1.
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.api.navigation

Function: velocity_set(self,vx, vy, vz, yaw_rate=0.0, tolerance=0.0, relative=False, async=False, yaw_rate_valid=False, body_frame=False):
        
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/velocity_set
call srv:
		:float vx
		:float vy
		:float vz
		:float yaw_rate
		:float tolerance
		:bool async
		:bool relative
		:bool yaw_rate_valid
		:bool body_frame
response srv: 
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/velocity_set
call srv:
		:float vx
		:float vy
		:float vz
		:float yaw_rate
		:float tolerance
    :bool async
    :bool relative
    :bool yaw_rate_valid
    :bool body_frame
response srv: 
    :bool success
    :string message

```

```javascript--REST
This is a REST call for the API to give velocity setpoints.
 Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/velocity_set'

JSON Request:
{   
    vx: Float,
    vy: Float,
    vz: Float,
    yaw_rate: Float,
    tolerance: Float,
    async: Boolean,
    relative: Boolean,
    yaw_rate_valid : Boolean,
    body_frame : Boolean }

JSON Response:
{   success: Boolean,
    message: String, }

```

```javascript--Websocket
This is a Websocket call for the API to give velocity setpoints.
 Make sure you initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/navigation/velocity_set',
serviceType: 'core_api/VelocitySet'

Request:
{       
    vx: Float,
    vy: Float,
    vz: Float,
    yaw_rate: Float,
    tolerance: Float,
    async: Boolean,
    relative: Boolean,
    yaw_valid : Boolean,
    body_frame : Boolean }

Response:
{   success: Boolean,
    message: String, }


```


> Example

```shell
rosservice call /flytpod/navigation/velocity_set "{vx: 0.5, vy: 0.2, vz: -0.1, yaw_rate: 0.1, tolerance: 0.0, async: false, relative: false, yaw_rate_valid: true, body_frame: false}"          

#sends (vx,vy,vz)=(0.5,0.2,-0.1)(m/s), yaw_rate=0.1rad/s,  relative=false, async=false, yaw_rate_valid=true, body_frame=false
#default value of tolerance=1.0m/s if left at 0  
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
nav.velocity_set(1.0, 0.5, -1.0, 0.12, 0.5, false, false, true, false);
//sends (vx,vy,vz)=(1.0,0.5,-1.0)(m/s), yaw_rate=0.12rad/s, tolerance=0.5m/s, relative=false, async=false, yaw_rate_valid=true, body_frame=false
```

```python
# create flyt_python navigation class instance
from flyt_python import api
drone = api.navigation()
# wait for interface to initialize
time.sleep(3.0)

# fly towards right ( with respect to vehicle current heading) 
drone.velocity_set(0, +2, 0, body_frame=True)

```

```cpp--ros
#include <core_api/PositionSet.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::PositionSet>("/<namespace>/navigation/position_set");
core_api::PositionSet srv;

srv.vx = 1.0;
srv.vy = 0.5;
srv.vz = -1.0;
srv.yaw_rate = 0.12;
srv.request.tolerance = 0.5;
srv.request.async = false;
srv.request.yaw_rate_valid = true;
srv.request.relative = false;
srv.request.body_frame = false;
client.call(srv);
success = srv.response.success;

//sends (vx,vy,vz)=(1.0,0.5,-1.0)(m/s), yaw_rate=0.12rad/s, tolerance=0.5m/s, relative=false, async=false, yaw_rate_valid=true, body_frame=false
```

```python--ros
import rospy
from core_api.srv import *

def setpoint_velocity(vx, vy, vz, yaw_rate, tolerance= 1.0, async = False, relative= False, yaw_rate_valid= False, body_frame= False):
    rospy.wait_for_service('/<namespace>/navigation/velocity_set')
    try:
        handle = rospy.ServiceProxy('/<namespace>/navigation/velocity_set', VelocitySet)
        # build message structure
        req_msg = VelocitySetRequest(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate, tolerance=tolerance, async=async, relative=relative, yaw_rate_valid=yaw_rate_valid, body_frame=body_frame)
        resp = handle(req_msg)
        
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("vel set service call failed %s", e)

```

```javascript--REST
var  msgdata={};
msgdata["vx"]=2.00;
msgdata["vy"]=3.00;
msgdata["vz"]=-1.00;
msgdata["yaw_rate"]=1.00;
msgdata["tolerance"]=2.00;
msgdata["async"]=true;
msgdata["relative"]=false;
msgdata["yaw_rate_valid"]=true;
msgdata["body_frame"]=false;

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/navigation/velocity_set",  
    success: function(data){
           console.log(data.success);
    }
};

```

```javascript--Websocket
var velocitySet = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/velocity_set',
    serviceType : 'core_api/VelocitySet'
});

var request = new ROSLIB.ServiceRequest({
    vx: 2.00,
    vy: 3.00,
    vz: -1.00,
    yaw_rate: 1.00,
    tolerance: 2.00,
    async: true,
    relative: false,
    yaw_rate_valid : true,
    body_frame : false
});

velocitySet.callService(request, function(result) {
    console.log('Result for service call on '
      + velocitySet.name
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

This API gives linear (vx,vy,vz) and angular (yaw_rate) velocity setpoint to vehicle. Please check API usage section below before using API.

###Parameters:
    
    Following parameters are applicable for onboard C++ and Python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    vx, vy, vz | float | Velocity Setpoint in NED-Frame (in body-frame if body_frame=true)
    yaw_rate | float | Yaw rate Setpoint in rad/sec
    yaw_rate_valid | bool | Must be set to true, if yaw 
    tolerance | float | Acceptance range in m/s, default value=1.0 m/s 
    relative | bool | If true, velocity setpoints relative to current position is sent
    async | bool | If true, asynchronous mode is set
    body_frame | bool | If true, velocity setpoints are relative with respect to body frame
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### ROS endpoint:
Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: Ros Service</br> 
* Name: /\<namespace\>/navigation/velocity_set</br>
* Service Type: core_api/VelocitySet

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ``POST http://<ip>/ros/<namespace>/navigation/velocity_set``
* JSON Request:
{
    vx: Float,
    vy: Float,
    vz: Float,
    yaw_rate: Float,
    tolerance: Float,
    async: Boolean,
    relative: Boolean,
    yaw_rate_valid : Boolean,
    body_frame : Boolean
}
* JSON Response:
{
    success: Boolean
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/\<namespace\>/navigation/velocity_set'</br>
* serviceType: 'core_api/VelocitySet'


### API usage information:

* Vehicle should be in GUIDED or OFFBOARD or API|POSCTL mode for this API to work.
* Vehicle should be armed for this API to work.
* Do not call this API when vehicle is grounded. Use take_off API first to get the vehicle in air.
* vx,vy,vz are velocity setpoints in 3 linear axes. Yaw rate is rate of angular rotation around Z axis. Right hand notation is used to find positive yaw direction.
* Effect of parameters:
  * Async:
     * True: The API call would return as soon as the command has been sent to the autopilot, irrespective of whether the vehicle has achieved the given velocity.
     * False: The API call would wait for the function to return, which happens when either the velocity setpoint is reached or timeout=30secs is over.
  * Relative: 
     * True: All setpoints (vx,vy,vz, yaw_rate) are calculated relative to current velocity. E.g. if vehicle is already flying in X direction with 1 m/s and a velocity call is placed for 2 m/s with relative= True then vehicle velocity target will change to 1+2 = 3 m/s.
     * False: All setpoints (vx,vy,vz, yaw) are used as absolute velocity setpoints.  
  * Body_frame 
     * True: All the setpoints are converted to body frame. 
        * Front of vehicle : +vx
        * Right of vehicle : +vy
        * Down: +vz
        * Yaw is calculated from front of vehicle. 
     * False: All the setpoints are converted to local NED (North, East, Down) frame. Yaw is calculated from North. 
* Either body_frame or relative flag can be set to true at a time. If both are set then only body_frame is effective.
* For yaw rate_ setpoint to be effective the yaw_rate_valid argument must be set to true.
* This API overrides any previous mission / navigation API being carried out.
* This API requires position lock. GPS, Optical Flow, VICON system can provide position data to vehicle.
* To provide only Yaw_rate setpoint use this API with x,y,z arguments set to 0, relative=True, yaw_valid=True
* Vehicle will keep flying in the direction specified by API if async= True. If async is False vehicle will stop after achieving target velocities in all directions. Make sure that you are handling such cases where vehicle might keep flying infinitely.
* Following parameters need to be manually configured according to vehicle frame.
  * MPC_XY_VEL_MAX : Maximum horizontal velocity. For smaller and lighter this parameter could be set to value between 8 m/s to 15 m/s. For larger and heavier systems it is safer to set this value below 8 m/s.
  * MPC_Z_VEL_MAX : Maximum vertical velocity. For smaller and lighter this parameter could be set to value between 3 m/s to 10 m/s. For larger and heavier systems it is safer to set this value below 8 m/s.
  * In any case vehicle will not exceed these velocity limits. So velocity_set call with target velocity beyond these limits will never be returned successful in synchronous mode.
  