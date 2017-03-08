# Position Setpoint Global

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
* Name: /namespace/navigation/position_set_global</br>
* Service Type: PositionSetGlobal

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

> Definition

```shell
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
Function Definition:     int position_set_global(float lat, float lon, float alt, float yaw=0, float tolerance=0, bool async=false, bool yaw_valid=false);
Arguments:
    lat, lon: Latitude and Longitude
    alt: Altitue (Positive distance upwards from home position)
    yaw: Yaw Setpoint in radians
    yaw_valid: Must be set to true, if yaw setpoint is provided
    tolerance: Acceptance radius in meters, default value=1.0m
    async: If true, asynchronous mode is set


Returns: For async=true, returns 0 if the command is successfully sent to the vehicle, else returns 1. For async=false, returns 0 if the vehicle reaches given setpoint before timeout=30secs, else returns 1.
```

```python

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

```

```javascript--REST

```

```javascript--Webocket

```


> Example API call

```shell
rosservice call /flytpod/navigation/position_set_global "{twist: {header: {seq: 0,stamp: {secs: 0, nsecs: 0}, frame_id: ''},twist: {linear: {x: 18.5204303, y:  73.8567437, z: -5.0}, angular: {x: 0.0, y: 0.0, z: 0.12}}}, tolerance: 0.0, async: false, yaw_valid: true}"

#sends (Lat,Lon,Alt)=(18.5204303, 73.8567437,5.0)(m), yaw=0.12rad, async=false, yaw_valid=true
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

```cpp
0
```

```python
```

```cpp--ros
success: True
```

```python--ros
```

```javascript--REST
{
}

```

```javascript--Websocket
{
}

```


