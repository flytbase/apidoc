## Position Hold

> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/position_hold
ROS-Service Type: core_api/PositionHold, below is its description

#Request : NULL

#Response : return success=true if command sent successfully to autopilot
bool success
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition:     int Navigation::position_hold()

Arguments: None

Returns:    returns 0 if the command is successfully sent to the vehicle
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.api.navigation

Function: position_hold():
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/position_hold
call srv: NULL
response srv: 
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/position_hold
call srv: NULL
response srv: 
    :bool success
    :string message

```

```javascript--REST
This is a REST call for the API to halt and hover at 
current location. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/position_hold'

JSON Response:
{   
    success: Boolean,
    message: String
}

```

```javascript--Websocket
This is a Websocket call for the API to halt and 
hover at current location. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/navigation/position_hold',
serviceType: 'core_api/PositionHold'

Response:
{   
    success: Boolean,
    message: String
}


```

> Example

```shell
rosservice call /<namespace>/navigation/position_hold "{}"
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
nav.position_hold();
```

```python
# create flyt_python navigation class instance
from flyt_python import api
drone = api.navigation()
# wait for interface to initialize
time.sleep(3.0)

# hold position
drone.position_hold()

```

```cpp--ros
#include <core_api/PositionHold.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::PositionHold>("/<namespace>/navigation/position_hold");
core_api::PositionHold srv;
client.call(srv);
bool success = srv.response.success;
std::string message = srv.response.message;
```

```python--ros
import rospy
from core_api.srv import *

def position_hold():
    rospy.wait_for_service('/<namespace>/navigation/position_hold')
    try:
        handle = rospy.ServiceProxy('/<namespace>/navigation/position_hold', PositionHold)
        resp = handle()
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("service call failed %s", e)

```

```javascript--REST

$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/navigation/position_hold",  
    success: function(data){
           console.log(data.success);
           console.log(data.message);
    }
});

```

```javascript--Websocket
var positionHold = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/position_hold',
    serviceType : 'core_api/PositionHold'
});

var request = new ROSLIB.ServiceRequest({});

positionHold.callService(request, function(result) {
    console.log('Result for service call on '
      + positionHold.name
      + ': '
      + result.success
      +': '
      + result.message);
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

Position hold / hover / loiter at current position.  

###Parameters:
    
    Following parameters are applicable for onboard C++ and Python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments: None
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### API usage information:

This API can be used to stop the vehicle at current location. 

* This API requires vehicle to be in GUIDED or OFFBOARD or API|POSCTL mode.
* This API will override current mission / navigation commmands. 
* This API requires position lock. GPS, Optical Flow, VICON system can provide position data to vehicle.
* Vehicle may take few seconds to come to rest depending on current linear velocity.

### ROS endpoint:

Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: `Ros Service`
* Name: `/<namespace>/navigation/position_hold`
* Service Type: `core_api/PositionHold`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port **80**. RESTful APIs can be called from remote platform of your choice.

* URL: `GET http://<ip>/ros/<namespace>/navigation/position_hold`
* JSON Response:
`{
    success: Boolean
    message: String
}`


### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 

Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace>/navigation/position_hold`
* serviceType: `core_api/PositionHold`
