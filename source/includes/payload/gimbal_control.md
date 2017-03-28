## Gimbal Control

> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

ROS-Service Name: /<namespace>/payload/gimbal_set
ROS-Service Type: core_api/GimbalSet, below is its description

#Request : expects gimbal attitude setpoint in radians via roll, pitch, yaw in NED Frame
#Response : return success=true if command is successfully sent

float64 roll
float64 pitch
float64 yaw
---
bool success
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

NotImplemented
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

NotImplemented
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/payload/gimbal_set()
call srv:
    :float64 roll
    :float64 pitch
    :float64 yaw
response srv: bool success
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Type: Ros Service
Name: /<namespace>/payload/gimbal_set()
call srv:
    :float64 roll
    :float64 pitch
    :float64 yaw
response srv: bool success
```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/payload/gimbal_set'

JSON Request:
{   roll: Float,
    pitch: Float,
    yaw: Float     }

JSON Response:
{   success: Boolean, }
```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/payload/gimbal_set',
serviceType: 'core_api/GimbalSet'

Request:
{   roll: Float,
    pitch: Float,
    yaw: Float }

Response:
{   success: Boolean, }
```

> Example

```shell
rosservice call /<namespace>/payload/gimbal_set "roll: 0.0
pitch:0.5
yaw:-0.2"

#sends (roll,pitch,yaw)=(0,0.5,-0.2)(rad)
```

```cpp
NotImplemented
```

```python
NotImplemented
```

```cpp--ros
#include <core_api/GimbalSet.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::GimbalSet>("payload/gimbal_set");
core_api::GimbalSet srv;

srv.request.roll = 0.0;
srv.request.pitch = 0.5;
srv.request.yaw = -0.2;
client.call(srv);
success = srv.response.success;
```

```python--ros
def setpoint_gimbal(roll, pitch, yaw):
    rospy.wait_for_service('<namespace>/payload/gimbal_set')
    try:
        handle = rospy.ServiceProxy('<namespace>/payload/gimbal_set', GimbalSet)
        resp = handle(roll, pitch, yaw)
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("gimbal_set service call failed %s", e)```
```

```javascript--REST
var  msgdata={};
msgdata["roll"]=0.0;
msgdata["pitch"]=0.5;
msgdata["yaw"]=-0.2;

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/payload/gimbal_set",  
    success: function(data){
           console.log(data.success);
    }
};
```

```javascript--Websocket
var gimbalSet = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/payload/gimbal_set',
    serviceType : 'core_api/GimbalSet'
});

var request = new ROSLIB.ServiceRequest({
    roll: 0.0,
    roll: 0.5,
    roll: -0.2
});

gimbalSet.callService(request, function(result) {
    console.log('Result for service call on '
      + gimbalSet.name
      + ': '
      + result.success);
});
```


> Example response

```shell
success: true
```

```cpp
NotImplemented
```

```python
NotImplemented
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
This API sends gimbal setpoint command to the autopilot via MAVLink and outputs pwm signals in gimbal-dedicated port of FlytPOD/Pixhawk. 

###Pre-requisites:
For this API to work, autopilot must fulfill some pre-requisites first:

* Autopilot **MUST** be in **ready-to-arm** state. Typically it would be reflected by RGBled patterns marked by either blue-breathing or green-breathing. For more information about autopilot RGBled patterns refer to [this link](http://docs.flytbase.com/docs/FlytPOD/Hardware_specifications.html#rgb-led).
* Make sure the parameter: **MNT_MODE_IN** is set to 3. 

###Parameters:
    
    Following parameters are applicable for onboard C++ and Python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    roll | float | roll command to gimbal in radians
    pitch | float | pitch command to gimbal in radians
    yaw | float | yaw command to gimbal in radians
    
    Output:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful

### ROS endpoint:
Payload APIs in FlytOS are derived from / wrapped around the core services available in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy api definition for message structure. 

* Type: Ros Service</br> 
* Name: /\<namespace\>/payload/gimbal_set</br>
* Service Type: GimbalSet

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ````POST http://<ip>/ros/<namespace>/payload/gimbal_set````
* JSON Request:
{   roll: Float,
    pitch: Float,
    yaw: Float
}
* JSON Response:
{
    success: Boolean
}

### Websocket endpoint:
Websocket APIs can be called from javascript using [roslibjs library](https://github.com/RobotWebTools/roslibjs).
Java websocket clients are supported using [rosjava](http://wiki.ros.org/rosjava).

* name: '/namespace/payload/gimbal_set'</br>
* serviceType: 'core_api/GimbalSet'

