# Namespace

> Make sure to replace \<namespace\> with your FlytOS namespace.

```shell
ROS-Service Name: /<namespace>/navigation/position_set
ROS-Service Type: core_api/PositionSet, below is its description

#Request : expects position setpoint via twist.twist.linear.x,linear.y,linear.z
#Request : expects yaw setpoint via twist.twist.angular.z (send yaw_valid=true)
geometry_msgs/TwistStamped twist
float32 tolerance
bool async
bool relative
bool yaw_valid
bool body_frame

#Response : success=true - (if async=false && if setpoint reached before timeout = 30sec) || (if async=true)
bool success
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

```

```python--ros

```

```shell--curl

```

```javascript--REST

```

```javascript--Webocket

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

```

```cpp--ros

```

```python--ros

```

```shell--curl

```

```javascript--REST

```

```javascript--Webocket

```


> Example API Response

```shell
success: true
```

```python

```

```cpp--ros

```

```python--ros

```

```shell--curl

```

```javascript--REST

```

```javascript--Webocket

```





----------a brief description of the API will come over here---------

This API sends position setpoint command to the autopilot. Additionally, you can send yaw setpoint (yaw_valid flag must be set true) to the vehicle as well. Some abstract features have been added, such as tolerance/acceptance-radius, synchronous/asynchronous mode, sending setpoints relative to current position (relative flag must be set true), sending setpoints relative to current body frame (body_frame flag must be set true).
This command commands the vehicle to go to a specified location and hover. It overrides any previous mission being carried out and starts hovering.

-------rest API doc will be here-------------

Over here we will define the REST endpoint API.