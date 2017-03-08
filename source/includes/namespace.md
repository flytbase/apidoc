# Namespace

> API definition

```shell
ROS-Service Name: /get_global_namespace
ROS-Service Type: core_api/ParamGetGlobalNamespace, below is its description

#Request : expects position setpoint via twist.twist.linear.x,linear.y,linear.z
#Request : expects yaw setpoint via twist.twist.angular.z (send yaw_valid=true)
geometry_msgs/TwistStamped twist
float32 tolerance
bool async
bool relative
bool yaw_valid
bool body_frame

#Response : Paramter info
core_api/ParamInfo param_info
#Response : success=true if parameter get was successfull.
bool success
#Response : Returns error message/success message if any.
string message
```

```cpp
Function Definition: int Navigation::position_set(float x, float y, float z, float yaw=0, float tolerance=0, bool relative=false, bool async=false, bool yaw_valid=false, bool body_frame=false)
Arguments:
	:param x,y,z: Position Setpoint in NED-Frame (in body-frame if body_frame=true)
	:param yaw: Yaw Setpoint in radians
	:param yaw_valid: Must be set to true, if yaw setpoint is provided
	:param tolerance: Acceptance radius in meters, default value=1.0m
	:param relative: If true, position setpoints relative to current position is sent
	:param async: If true, asynchronous mode is set
	:param body_frame: If true, position setpoints are relative with respect to body frame
	:return: For async=true, returns 0 if the command is successfully sent to the vehicle, else returns 1. For async=false, returns 0 if the vehicle reaches given setpoint before timeout=30secs, else returns 1.
```

```python
Class: flyt_python.api.navigation
Function Definition: get_global_namespace()
Arguments: None
return: string
```

```cpp--ros
Function Definition: void ros::param::get("/global_namespace", string global_namespace)
Arguments:
```

```python--ros
Type: Ros Service
Name: /get_global_namespce()
response srv: ParamGetGlobalNamespace

```

```javascript--REST
URL: ' <ip>/ros/get_global_namespace'

JSON Response:
	{
		success: Boolean
		param_info:{
			param_value: String
		}
	}

```

```javascript--Websocket

```


> Example API call

```shell
rosservice call /<namespace>/navigation/position_set "twist:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  twist:
    linear: {x: 1.0, y: 3.5, z: -5.0}
    angular: {x: 0.0, y: 0.0, z: 0.5}
tolerance: 0.0
async: false
relative: false
yaw_valid: true
body_frame: false"

#sends (x,y,z)=(1.0,3.5,-5.0)(m), yaw=0.12rad, relative=false, async=false, yaw_valid=true, body_frame=false
#default value of tolerance=1.0m if left at 0    
```

```python
from flyt_python import api
drone = api.navigation()
time.sleep(3.0)
namespace = drone.get_global_namespace()

```

```cpp--ros

```

```python--ros
def get_global_namespace():
    rospy.wait_for_service('/get_global_namespace')
    try:
        res = rospy.ServiceProxy('/get_global_namespace', ParamGetGlobalNamespace)
        op = res()
        return str(op.param_info.param_value)
    except rospy.ServiceException, e:
        rospy.logerr("global namespace service not available", e)
        return None

```


```javascript--REST
	$.ajax({
	    type: "GET",
	    dataType: "json",
	    url: "http://<ip>/ros/get_global_namespace",   
	    success: function(data){
	        console.log(data);
	    }
	});


```

```javascript--Webocket

```


> Example API Response

```shell
success: true
```

```python
flytpod
```

```cpp--ros
param_info: 
  param_id: global_namespace
  param_value: flytpod
success: True
message: Parameter Get Global Namespace Successful	flytpod
```

```python--ros
Response object with following structure.
param_info: 
  param_id: global_namespace
  param_value: flytpod
success: True
message: Parameter Get Global Namespace Successful	flytpod
```


```javascript--REST
	data:{
		success:True,
		param_info:{
			param_value:'flytpod'
		}
	}

```

```javascript-Websocket

```





###Description:
This API sends local position setpoint command to the autopilot. Additionally, you can send yaw setpoint (yaw_valid flag must be set true) to the vehicle as well. Some abstract features have been added, such as tolerance/acceptance-radius, synchronous/asynchronous mode, sending setpoints relative to current position (relative flag must be set true), sending setpoints relative to current body frame (body_frame flag must be set true).
This command commands the vehicle to go to a specified location and hover. It overrides any previous mission being carried out and starts hovering.

###Parameters:
    
    Following parameters are applicable for onboard C++ and Python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
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
* Name: /namespace/navigation/position_set</br>
* Service Type: PositionSet

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ````GET http://<ip>/ros/<namespace>/navigation/disarm````
* JSON Response:
{
    success: Boolean
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/navigation/disarm'</br>
* serviceType: 'core_api/Disarm'


### API usage information:
Note: You can either set body_frame or relative flag. If both are set, body_frame takes precedence.

Tip: Asynchronous mode - The API call would return as soon as the command has been sent to the autopilot, irrespective of whether the vehicle has reached the given setpoint or not.

Tip: Synchronous mode - The API call would wait for the function to return, which happens when either the position setpoint is reached or timeout=30secs is over.

