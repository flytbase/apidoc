## Set Waypoints

> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/waypoint_set
ROS-Service Type: core_api/WaypointSet, below is its description

# Request: Waypoints to be sent to device
mavros_msgs/Waypoint[] waypoints

# Returns: success status and transfered count
bool success
string message
uint32 wp_transfered

```

```cpp
// CPP API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from cpp.

Function Definition:    int Navigation::waypoint_set(std::vector<mavros_msgs::Waypoint> waypoints)

Arguments:
    waypoint: Array of waypoints to be sent to the autopilot

Returns:    0 if the land command is successfully sent to the vehicle, else returns 1.

```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.
Class: flyt_python.API.navigation

Function: waypoint_set(waypoints)

```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/waypoint_set
call srv:
    :mavros_msgs/Waypoint[] waypoints
response srv: 
    :bool success
    :uint32 wp_transfered
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/waypoint_set
call srv:
    :mavros_msgs/Waypoint[] waypoints
response srv: 
    :bool success
    :uint32 wp_transfered

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/waypoint_set'

JSON Request:
{   waypoints:[{
        frame : [Int] 0/1/2/3/4,
        command : [Int] 16/17/18/19/20/21/22,
        is_current : [Boolean],
        autocontinue : [Boolean],
        param1 : [Float],
        param2 : [Float],
        param3 : [Float],
        param4 : [Float],
        x_lat : [Float],
        y_long : [Float],
        z_alt : [Float],
        },{},{}... ] }

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

name: '/<namespace>/navigation/waypoint_set',
serviceType: 'core_api/WaypointSet'

Request:
{   waypoints:[{
        frame : [Int] 0/1/2/3/4,
        command : [Int] 16/17/18/19/20/21/22,
        is_current : [Boolean],
        autocontinue : [Boolean],
        param1 : [Float],
        param2 : [Float],
        param3 : [Float],
        param4 : [Float],
        x_lat : [Float],
        y_long : [Float],
        z_alt : [Float],
        },{},{}... ] }

Response:
{   success: Boolean,
    message: String, }

```

> Example

```shell
  rosservice call /flytos/navigation/waypoint_set "waypoints:
- {frame: 0, command: 0, is_current: false, autocontinue: false, param1: 0.0, param2: 0.0,
  param3: 0.0, param4: 0.0, x_lat: 0.0, y_long: 0.0, z_alt: 0.0}" 

```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
mavros_msgs::Waypoint waypoint;
std::vector<mavros_msgs::Waypoint> waypoints_array;
waypoint.frame = 3;
waypoint.command = 16;
waypoint.is_current = false;
waypoint.autocontinue = true;
waypoint.param1 = 0; 
waypoint.param2 = 1;
waypoint.param3 = 0;
waypoint.param4 = 0;
waypoint.x_lat = 73.2154;
waypoint.x_long = 18.5472;
waypoint.z_alt = 5;
waypoints_array.pushback(waypoint)
nav.waypoint_set(waypoints_array);
```

```python
# create flyt_python navigation class instance
from flyt_python import API
drone = API.navigation()
# wait for interface to initialize
time.sleep(3.0)

#for multiple waypoints
#list of waypoints to be constructed
wp4 =  [{'frame': 0,
       'command': 16,
       'is_current': True,
       'autocontinue': True,
       'param1': 10.2,
       'param2': 10.2,
       'param3': 10.2,
       'param4': 10.2,
       'x_lat': x - 0.0002,
       'y_long': y - 0.0002,
       'z_alt': z + 10
       },
      {'frame': 0,
       'command': 16,
       'is_current': True,
       'autocontinue': True,
       'param1': 10.2,
       'param2': 10.2,
       'param3': 10.2,
       'param4': 10.2,
       'x_lat': x+0.0001,
       'y_long': y + 0.0001,
       'z_alt': z + 10
       },
      {'frame': 0,
       'command': 16,
       'is_current': True,
       'autocontinue': True,
       'param1': 10.2,
       'param2': 10.2,
       'param3': 10.2,
       'param4': 10.2,
       'x_lat': x,
       'y_long': y,
       'z_alt': z + 10
       },
       {'frame': 0,
        'command': 16,
        'is_current': True,
        'autocontinue': True,
        'param1': 10.2,
        'param2': 10.2,
        'param3': 10.2,
        'param4': 10.2,
        'x_lat': x + 0.0001,
        'y_long': y + 0.0001,
        'z_alt': z + 10
       }]

# set list of current waypoints  
drone.waypoint_set(wp4)

#for single waypoint
wp1 = {'frame': 0,
       'command': 16,
       'is_current': True,
       'autocontinue': True,
       'param1': 10.2,
       'param2': 10.2,
       'param3': 10.2,
       'param4': 10.2,
       'x_lat': x - 0.0002,
       'y_long': y - 0.0002,
       'z_alt': z + 10
       }
#set list of current waypoints
drone.waypoint_set(wp1)
```

```cpp--ros
// Please refer to Roscpp documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c%2B%2B)
```

```python--ros

# Please refer to Rospy documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)

```

```javascript--REST
var  msgdata=[];
msgdata[1]={};
msgdata[1]["frame"]=3;
msgdata[1]["command"]= 16;
msgdata[1]["is_current"]= false;
msgdata[1]["autocontinue"]= true;
msgdata[1]["param1"]= 0;
msgdata[1]["param2"]= 1;
msgdata[1]["param3"]= 0;
msgdata[1]["param4"]= 0;
msgdata[1]["x_lat"]= 73.2154;
msgdata[1]["y_long"]= 18.5472;
msgdata[1]["z_alt"]= 5;

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/navigation/waypoint_set",  
    success: function(data){
           console.log(data.success);
           console.log(data.message);
    }
};

```

```javascript--Websocket
var waypointSet = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/waypoint_set',
    serviceType : 'core_api/WaypointSet'
});

var request = new ROSLIB.ServiceRequest({
    waypoints:[{
        frame : [Int] 0/1/2/3/4,
        command : [Int] 16/17/18/19/20/21/22,
        is_current : [Boolean],
        autocontinue : [Boolean],
        param1 : [Float],
        param2 : [Float],
        param3 : [Float],
        param4 : [Float],
        x_lat : [Float],
        y_long : [Float],
        z_alt : [Float],
        },{},{}... ]
});

waypointSet.callService(request, function(result) {
    console.log('Result for service call on '
      + waypointSet.name
      + ': '
      + result.success
      +': '
      + result.message);
});
```


> Example response

```shell
success: True
wp_transfered: 0
```

```cpp
0
```

```python
{'message': '[INFO] Waypoint set Successful', 'wp_transfered': 4, 'success': True}


wp_transfered (int): Number of waypoints transfered
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

### Description:

This API replaces current list of waypoints on autopilot with new list passed.

### Parameters:
    
    Following parameters are applicable in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
     Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
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
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### ROS endpoint:

Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: `Ros Service`
* Name: `/<namespace>/navigation/waypoint_set`
* Service Type: `WaypointSet`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: `POST http://<ip>/ros/<namespace>/navigation/waypoint_set`
* JSON Request:
`{
    waypoints:[{
        frame : [Int] 0/1/2/3/4,
        command : [Int] 16/17/18/19/20/21/22,
        is_current : [Boolean],
        autocontinue : [Boolean],
        param1 : [Float],
        param2 : [Float],
        param3 : [Float],
        param4 : [Float],
        x_lat : [Float],
        y_long : [Float],
        z_alt : [Float],
        },{},{}... ]
}`
* JSON Response:
`{
    success: Boolean
    message: String
}`

### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 

Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace>/navigation/waypoint_set`
* serviceType: `core_api/WaypointSet`
