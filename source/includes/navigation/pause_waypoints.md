## Pause Waypoints


> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/waypoint_pause
ROS-Service Type: core_api/WaypointPause, below is its description

#Request : Null

#Response : success = true if command sent successfully
bool success
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition: int Navigation::waypoint_pause(void)

Arguments:  None

Returns:    returns 0 if the command is successfully sent to the vehicle
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.API.navigation

Function:  waypoint_pause()
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/waypoint_pause
call srv: NULL
response srv: 
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/waypoint_pause
call srv: NULL
response srv: 
    :bool success
    :string message

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/waypoint_pause'

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

name: '/<namespace>/navigation/waypoint_pause',
serviceType: 'core_api/WaypointPause'

Response:
{   success: Boolean,
    message: String, }


```


> Example

```shell
rosservice call /flytpod/navigation/waypoint_pause "{}"      
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
nav.waypoint_pause();
```

```python
# create flyt_python navigation class instance
from flyt_python import API
drone = API.navigation()
# wait for interface to initialize
time.sleep(3.0)

#pause the waypoints mission
drone.waypoint_pause()
```

```cpp--ros
// Please refer to Roscpp documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c%2B%2B)
```

```python--ros

# Please refer to Rospy documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)

```

```javascript--REST

$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/navigation/waypoint_pause",  
    success: function(data){
           console.log(data.success);
           console.log(data.message);
    }
};

```

```javascript--Websocket
var waypointPause = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/waypoint_pause',
    serviceType : 'core_api/WaypointPause'
});

var request = new ROSLIB.ServiceRequest({});

waypointPause.callService(request, function(result) {
    console.log('Result for service call on '
      + waypointPause.name
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
{'message': '[INFO] Waypoint Paused', 'success': True}

message (string): Contains error message
success (bool): true if action successful
```

```cpp--ros
```

```python--ros
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

This API pauses ongoing waypoint mission.
###Parameters:
    
    Following parameters are applicable in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### ROS endpoint:
Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: Ros Service</br> 
* Name: /\<namespace\>/navigation/waypoint_pause</br>
* Service Type: WaypointPause

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ``GET http://<ip>/ros/<namespace>/navigation/waypoint_pause``
* JSON Response:
{
    success: Boolean
    message: String
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/\<namespace\>/navigation/waypoint_pause'</br>
* serviceType: 'core_api/WaypointPause'



