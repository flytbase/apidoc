## Set Current Waypoint


> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/waypoint_set_current
ROS-Service Type: core_api/WaypointSetCurrent, below is its description

# Request: set current waypoint to index ( wp_seq ) in waypoint array
uint16 wp_seq
# Response: success if command sent successfully
bool success
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition: int Navigation::waypoint_set_current(int waypoint_no)

Arguments:  
    waypoint_no: Index of waypoint to be set as current waypoint

Returns:    returns 0 if the command is successfully sent to the vehicle
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from python.

Class: flyt_python.API.navigation

Function:  waypoint_set_current(wp_seq)
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/waypoint_set_current
call srv: uint16 wp_seq
response srv: 
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/waypoint_set_current
call srv: uint16 wp_seq
response srv: 
    :bool success
    :string message

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/waypoint_set_current'

JSON Request:
{
    wp_seq: Int
}

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

name: '/<namespace>/navigation/waypoint_set_current',
serviceType: 'core_api/WaypointSetCurrent'

Request:
{
    wp_seq: Int
}

Response:
{
    success: Boolean,
    message: String
}


```

```python--flyt_python

# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.flyt_python.DroneApiConnector

Function: set_current_waypoint(wp_seq)

```

> Example

```shell
rosservice call /flytsim/navigation/waypoint_set_current "wp_seq: 1" 
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
int waypoint_no = 2;
nav.waypoint_set_current(waypoint_no);
```

```python
# create flyt_python navigation class instance
from flyt_python import API
drone = API.navigation()
# wait for interface to initialize
time.sleep(3.0)

#setting the 2nd waypoint as the current waypoint
drone.waypoint_set_current(2)
```

```cpp--ros
// Please refer to Roscpp documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c%2B%2B)
```

```python--ros

# Please refer to Rospy documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)

```

```javascript--REST
var  msgdata={};
msgdata["wp_seq"]=2;

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/navigation/waypoint_set_current",  
    success: function(data){
           console.log(data.success);
           console.log(data.message);
    }
});

```

```javascript--Websocket
var waypointSetCurrent = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/waypoint_set_current',
    serviceType : 'core_api/WaypointSetCurrent'
});

var request = new ROSLIB.ServiceRequest({
    wp_seq: 2
});

waypointSetCurrent.callService(request, function(result) {
    console.log('Result for service call on '
      +waypointSetCurrent.name
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
    
drone.set_current_waypoint(2)
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
{'message': '[INFO] Waypoint set_current Successful', 'success': True}

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

Sets the waypoint Id specified, as the current waypoint from the list of already set wayopints.

### Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    wp_seq | int | Id of the waypoint fro  the list of set waypoints.

    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### API usage information:

<aside class="notice">
    Make sure you already have a waypoint list set beforehand executing this API for it to work.
</aside>

### ROS endpoint:

Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure.

* Type: `Ros Service`
* Name: `/<namespace>/navigation/waypoint_set_current`
* Service Type: `WaypointSetCurrent`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: `POST http://<ip>/ros/<namespace>/navigation/waypoint_set_current`
* JSON Request:
`{
    wp_seq: Int
}`
* JSON Response:
`{
    success: Boolean,
    message: String
}`


### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 

Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace>/navigation/waypoint_set_current`
* serviceType: `core_api/WaypointSetCurrent`
