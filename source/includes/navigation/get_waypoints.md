## Get Waypoints

> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/waypoint_get
ROS-Service Type: core_api/WaypointGet, below is its description

# Request: NULL

# Returns success status and received count
bool success
string message
uint32 wp_received
mavros_msgs/Waypoint[] waypoints

```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition: int Navigation::waypoint_get(std::vector<mavros_msgs::Waypoint> &waypoints)

Arguments:  waypoints: list of waypoints will be made available in this variable

Returns:    returns 0 if the command is successfully sent to the vehicle
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.
Class: flyt_python.API.navigation

Function: waypoint_get()
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/waypoint_get
call srv: Null
response srv: 
    :bool success
    :uint32 wp_received
    :mavros_msgs/Waypoint[] waypoints
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/waypoint_get
call srv: Null
response srv: 
    :bool success
    :uint32 wp_received
    :mavros_msgs/Waypoint[] waypoints
```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/waypoint_get'

JSON Response:
{   success: Boolean,
    message: String, 
    wp_recieved: Int,
    waypoints: [{
        frame: Int 0/1/2/3/4,
        command:Int 16/17/18/19/20/21/22,
        is_current: Boolean,
        autocontinue: Boolean,
        param1: Float,
        param2: Float,
        param3: Float,
        param4: Float,
        x_lat: Float,
        y_long: Float,
        z_alt: Float},{},{}...
    ]
}

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/navigation/waypoint_get',
serviceType: 'core_api/WaypointGet'

Response:
{   success: Boolean,
    message: String, 
    wp_recieved: Int,
    waypoints: [{
        frame: Int 0/1/2/3/4,
        command:Int 16/17/18/19/20/21/22,
        is_current: Boolean,
        autocontinue: Boolean,
        param1: Float,
        param2: Float,
        param3: Float,
        param4: Float,
        x_lat: Float,
        y_long: Float,
        z_alt: Float},{},{}...
    ]
}


```

> Example

```shell
rosservice call /flytos/navigation/waypoint_get "{}" 
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
std::vector<mavros_msgs::Waypoint> &waypoints
nav.waypoint_get(waypoints);
std::cout<<"Number of waypoints received\t"<<waypoints.size()<<"\n";
```

```python
# create flyt_python navigation class instance
from flyt_python import API
drone = API.navigation()
# wait for interface to initialize
time.sleep(3.0)

# get list of current waypoints  
wp = drone.waypoint_get()

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
    url: "http://<ip>/ros/<namespace>/navigation/waypoint_get",  
    success: function(data){
           console.log(data.waypoints);
    }
};

```

```javascript--Websocket
var waypointGet = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/waypoint_get',
    serviceType : 'core_api/WaypointGet'
});

var request = new ROSLIB.ServiceRequest({});

waypointGet.callService(request, function(result) {
    console.log('Result for service call on '
      + waypointGet.name
      + ': '
      + result.success
      +': '
      + result.message);
});
```

> Example response

```shell
success: True
wp_received: 1
waypoints: 
  - 
    frame: 3
    command: 16
    is_current: True
    autocontinue: True
    param1: 0.0
    param2: 0.0
    param3: 0.0
    param4: 0.0
    x_lat: 18.6204299927
    y_long: 73.903465271
    z_alt: 50.0
```

```cpp
0
```

```python
{'wp_received': 2, 'message': '[INFO] Waypoint get Successful', 'success': True, 'waypoints': [{'autocontinue': True, 'frame': 0, 'command': 16, 'param4': 10.199999809265137, 'is_current': True, 'param2': 10.199999809265137, 'param1': 10.199999809265137, 'y_long': -122.08358764648438, 'param3': 0.0, 'x_lat': 7.429123401641846, 'z_alt': 112.61199951171875}, {'autocontinue': True, 'frame': 0, 'command': 16, 'param4': 10.199999809265137, 'is_current': False, 'param2': 10.199999809265137, 'param1': 10.199999809265137, 'y_long': -122.08329010009766, 'param3': 0.0, 'x_lat': 7.4294233322143555, 'z_alt': 112.61199951171875}]}


wp_received (int): Number of waypoints received
message (string): Contains error message
success (bool): true if action successful
waypoints (list): consists a list of dictionary, the dictionary consists of (frame, command, is_current, autocontinue, param1, param2, param3, param4, x_lat, y_long, z_alt)
```

```cpp--ros
```

```python--ros
```

```javascript--REST
{
    success: True, 
    wp_recieved: 2,
    waypoints: [{
        frame: 3,
        command:Int 16,
        is_current: true,
        autocontinue: true,
        param1: 6.0,
        param2: 7.0,
        param3: 0.0,
        param4: 0.0,
        x_lat: 65.425532,
        y_long: 18.542422,
        z_alt: 25},{}]
}

```

```javascript--Websocket
{
    success: True, 
    wp_recieved: 2,
    waypoints: [{
        frame: 3,
        command:Int 16,
        is_current: true,
        autocontinue: true,
        param1: 6.0,
        param2: 7.0,
        param3: 0.0,
        param4: 0.0,
        x_lat: 65.425532,
        y_long: 18.542422,
        z_alt: 25},{}]
}

```

###Description:

This API returns list of current waypoints on autopilot.

###Parameters:
    
    Following parameters are applicable RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments: None

    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message
    frame | int | The Frame in which the waypoints are given<br>0: Global<br>1: Local NED<br>2: Mission<br>3: Global Rel Alt
    command | int | defines the function of the waypoint<br>16: Waypoints<br>17: Loiter<br>18: Loiter Turns<br>19: Loiter time<br>20: Return to Launch<br>21: Land<br>22: Take Off
    is_current | bool | Set it as the first waypoint 
    autocontinue | bool | continue to the next waypoint as soon as the current waypoint is achieved
    param1 | float | Time to stay at the location in sec.
    param2 | float | radius around the waypoint within which the waypoint is marked as done 
    param3 | float | Orbit radius in meters
    param4 | float | Yaw/direction in degrees
    x_lat | float | Latitude
    y_long | float | Longitude
    z_alt | float | Relative altitude

<aside class="notice">
    See actual response body on the right.
</aside>

### ROS endpoint:

Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: `Ros Service`
* Name: `/<namespace>/navigation/waypoint_get`
* Service Type: `core_api/WaypointGet`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port **80**. RESTful APIs can be called from remote platform of your choice.

* URL: `GET http://<ip>/ros/<namespace>/navigation/waypoint_get`
* JSON Response:
`{
    success: Boolean,
    message: String, 
    wp_recieved: Int,
    waypoints: [{
        frame: Int 0/1/2/3/4,
        command:Int 16/17/18/19/20/21/22,
        is_current: Boolean,
        autocontinue: Boolean,
        param1: Float,
        param2: Float,
        param3: Float,
        param4: Float,
        x_lat: Float,
        y_long: Float,
        z_alt: Float},{},{}...] }
}`

### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 

Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace>/navigation/waypoint_get`
* serviceType: `core_api/WaypointGet`
