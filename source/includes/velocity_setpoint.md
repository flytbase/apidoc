# Velocity Setpoint


> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/velocity
ROS-Service Type: core_api/VelocitySet, below is its description

#Request : expects velocity setpoint via twist.twist.linear.x,linear.y,linear.z
#Request : expects yaw_rate setpoint via twist.twist.angular.z (send yaw_rate_valid=true)
geometry_msgs/TwistStamped twist
float32 tolerance
bool async
bool relative
bool yaw_rate_valid
bool body_frame

#Response : return success=true, (if async=false && if setpoint reached before timeout = 30sec) || (if async=true && command sent to autopilot)
bool success
```

```cpp
// CPP API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from cpp.

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
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from python.

Class: flyt_python.api.navigation

Function: position_set(self, x, y, z, yaw=0.0, tolerance=0.0, relative=False, async=False, yaw_valid=False,
                     body_frame=False):
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/velocity_set()
call srv:
    :geometry_msgs/TwistStamped twist
    :float32 tolerance
    :bool async
    :bool relative
    :bool yaw_rate_valid
    :bool body_frame
response srv: bool success
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/position_set()
call srv:
    :geometry_msgs/TwistStamped twist
    :float32 tolerance
    :bool async
    :bool relative
    :bool yaw_valid
    :bool body_frame
response srv: bool success

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/position_set'

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

name: '/<namespace>/navigation/position_set',
serviceType: 'core_api/PositionSet'

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
    yaw_valid : Boolean,
    body_frame : Boolean }

Response:
{   success: Boolean, }


```


> Example

```shell
rosservice call /flytpod/navigation/velocity_set "{twist: {header: {seq: 0,stamp: {secs: 0, nsecs: 0}, frame_id: ''},twist: {linear: {x: 0.5, y: 0.2, z: -0.1}, angular: {x: 0.0, y: 0.0, z: 0.1}}}, tolerance: 0.0, async: false, relative: false, yaw_rate_valid: true, body_frame: false}"          

#sends (vx,vy,vz)=(0.5,0.2,-0.1)(m/s), yaw_rate=0.1rad/s,  relative=false, async=false, yaw_rate_valid=true, body_frame=false
#default value of tolerance=1.0m/s if left at 0  
```

```cpp
#include <core_script_bridge/navigation_bridge.h>

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

# command vehicle towards 5 meteres WEST from current location regardless of heading
drone.position_set(-5, 0, 0, relative=True)

```

```cpp--ros
#include <core_api/PositionSet.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::PositionSet>("navigation/position_set");
core_api::PositionSet srv;

srv.request.twist.twist.angular.z = 0.12;
srv.request.twist.twist.linear.x = 1.0;
srv.request.twist.twist.linear.y = 0.5;
srv.request.twist.twist.linear.z = -1.0;
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
def setpoint_local_position(lx, ly, lz, yaw, tolerance= 0.0, async = False, relative= False, yaw_rate_valid= False, body_frame= False):
    rospy.wait_for_service('namespace/navigation/position_set')
    try:
        handle = rospy.ServiceProxy('namespace/navigation/position_set', PositionSet)
        twist = {'header': {'seq': seq, 'stamp': {'secs': sec, 'nsecs': nsec}, 'frame_id': f_id}, 'twist': {'linear': {'x': lx, 'y': ly, 'z': lz}, 'angular': {'z': yaw}}}
        resp = handle(twist, tolerance, async, relative, yaw_rate_valid, body_frame)
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("pos set service call failed %s", e)

```

```javascript--REST
var  msgdata={};
msgdata["twist"]={};
msgdata.twist["twist"]={};
masdata.twist.twist["linear"]={};
msgdata.twist.twist.linear["x"]=2.00;
msgdata.twist.twist.linear["y"]=3.00;
msgdata.twist.twist.linear["z"]=-1.00;
msgdata.twist.twist["angular"]={};
msgdata.twist.twist.angular["z"]=1.00;
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
    }
};

```

```javascript--Websocket
var positionSet = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/position_set',
    serviceType : 'core_api/PositionSet'
});

var request = new ROSLIB.ServiceRequest({
    twist:{twist:{  linear:{
                x: 2.00,
                y: 3.00,
                z: -1.00
            },angular:{
                z: 1.00
    }}},
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
This API sends local position setpoint command to the autopilot. Additionally, you can send yaw setpoint (yaw_valid flag must be set true) to the vehicle as well. Some abstract features have been added, such as tolerance/acceptance-radius, synchronous/asynchronous mode, sending setpoints relative to current position (relative flag must be set true), sending setpoints relative to current body frame (body_frame flag must be set true).
This command commands the vehicle to go to a specified location and hover. It overrides any previous mission being carried out and starts hovering.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTFul, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
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
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful

### ROS endpoint:
Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy api definition for message structure. 

* Type: Ros Service</br> 
* Name: /namespace/navigation/velocity_set</br>
* Service Type: VelocitySet

### RESTFul endpoint:
FlytOS hosts a RESTFul server which listens on port 80. RESTFul APIs can be called from remote platform of your choice.

* URL: ````POST http://<ip>/ros/<namespace>/navigation/position_set````
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
    yaw_valid : Boolean,
    body_frame : Boolean
}
* JSON Response:
{
    success: Boolean
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/navigation/position_set'</br>
* serviceType: 'core_api/PositionSet'


### API usage information:
Note: You can either set body_frame or relative flag. If both are set, body_frame takes precedence.

Tip: Asynchronous mode - The API call would return as soon as the command has been sent to the autopilot, irrespective of whether the vehicle has reached the given setpoint or not.

Tip: Synchronous mode - The API call would wait for the function to return, which happens when either the position setpoint is reached or timeout=30secs is over.

