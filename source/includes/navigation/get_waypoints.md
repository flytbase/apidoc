## Get Waypoints


> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/waypoint_get
ROS-Service Type: core_api/WaypointGet, below is its description

# Request: NULL

# Returns success status and received count
bool success
uint32 wp_received
mavros_msgs/Waypoint[] waypoints

```

```cpp

```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

NotImplemented
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/waypoint_get()
call srv: Null
response srv: 
    :bool success
    :uint32 wp_received
    :mavros_msgs/Waypoint[] waypoints
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/waypoint_get()
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
        z_alt: Float},{},{}...]
}

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/navigation/waypoint_get',
serviceType: 'core_api/WaypointGet'

Response:
{   success: Boolean, 
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


```


> Example

```shell
rosservice call /flytpod/navigation/waypoint_get "{}" 
```

```cpp

```

```python

NotImplemented

```

```cpp--ros
// Please refer to Roscpp documenation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c%2B%2B)
```

```python--ros

# Please refer to Rospy documenation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)

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
      + result.success);
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
```

```python
NotImplemented
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
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    frame | int | The Frame in which the waypoints are given<br>0: Global<br>1:Local NED<br>2: Mission<br>3:Global Rel Alt
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
* Name: /namespace/navigation/waypoint_get</br>
* Service Type: core_api/WaypointGet

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ````GET http://<ip>/ros/<namespace>/navigation/waypoint_get````
* JSON Response:
{
    success: Boolean, 
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
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/navigation/waypoint_get'</br>
* serviceType: 'core_api/WaypointGet'


### API usage information:
Note: You can either set body_frame or relative flag. If both are set, body_frame takes precedence.

Tip: Asynchronous mode - The API call would return as soon as the command has been sent to the autopilot, irrespective of whether the vehicle has reached the given setpoint or not.

Tip: Synchronous mode - The API call would wait for the function to return, which happens when either the position setpoint is reached or timeout=30secs is over.

