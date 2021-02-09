## Execute Waypoints

> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/waypoint_execute
ROS-Service Type: core_api/waypointExecute, below is its description

#Request : Null

#Response : success = true if command sent successfully
bool success
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition: int Navigation::waypoint_execute(void)

Arguments:  None

Returns:    returns 0 if the command is successfully sent to the vehicle
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.
Class: flyt_python.API.navigation

Function: drone.waypoint_execute()
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/waypoint_execute
call srv: NULL
response srv: 
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/waypoint_execute
call srv: NULL
response srv: 
    :bool success
    :string message

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/waypoint_execute'

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

name: '/<namespace>/navigation/waypoint_execute',
serviceType: 'core_api/WaypointExecute'

Response:
{   success: Boolean,
    message: String, }


```

```python--flyt_python

# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.flyt_python.DroneApiConnector

Function: execute_waypoints()

```

> Example

```shell
rosservice call /flytsim/navigation/waypoint_execute "{}"   
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
nav.waypoint_execute();
```

```python
# create flyt_python navigation class instance
from flyt_python import API
drone = API.navigation()
# wait for interface to initialize
time.sleep(3.0)

#execute the waypoints
drone.waypoint_execute()
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
    url: "http://<ip>/ros/<namespace>/navigation/waypoint_execute",  
    success: function(data){
           console.log(data.success);
           console.log(data.message);
    }
});

```

```javascript--Websocket
var waypointExecute = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/waypoint_execute',
    serviceType : 'core_api/WaypointExecute'
});

var request = new ROSLIB.ServiceRequest({});

waypointExecute.callService(request, function(result) {
    console.log('Result for service call on '
      + waypointExecute.name
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
    
drone.execute_waypoints()

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
{'message': '[INFO] Waypoint Execution initiated', 'success': True}

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
```python--flyt_python
{
    success: True, 
    message: message

}
```

### Description:

Exectute / resume current list of waypoints.

### Parameters:
    
    Following parameters are applicable in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### API usage information:

Note: Make sure you have a list of waypoints already set using `set_waypoints` API before you give it execute_waypoint API call.

### ROS endpoint:

Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: `Ros Service`
* Name: `/<namespace>/navigation/waypoint_execute`
* Service Type: `WaypointExecute`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: `GET http://<ip>/ros/<namespace>/navigation/waypoint_execute`
* JSON Response:
`{
    success: Boolean
    message: String
}`

### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 

Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace>/navigation/waypoint_execute`
* serviceType: `core_api/WaypointExecute`
