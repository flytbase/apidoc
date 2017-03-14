# Set Waypoints


> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/waypoint_set
ROS-Service Type: core_api/WaypointSet, below is its description

# Request: Waypoints to be sent to device
mavros_msgs/Waypoint[] waypoints

# Returns: success status and transfered count
bool success
uint32 wp_transfered

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
Name: /<namespace>/navigation/waypoint_set()
call srv:
    :mavros_msgs/Waypoint[] waypoints
response srv: 
    :bool success
    :uint32 wp_transfered
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/waypoint_set()
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
{   success: Boolean, }

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
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
{   success: Boolean, }


```


> Example

```shell
  rosservice call /flytpod/navigation/waypoint_set "waypoints:
- {frame: 0, command: 0, is_current: false, autocontinue: false, param1: 0.0, param2: 0.0,
  param3: 0.0, param4: 0.0, x_lat: 0.0, y_long: 0.0, z_alt: 0.0}" 

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
msgdata[1]["z_lat"]= 5;

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/navigation/waypoint_set",  
    success: function(data){
           console.log(data.success);
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
      + result.success);
});
```


> Example response

```shell
success: True
wp_transfered: 0
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
    success:True
}

```

```javascript--Websocket
{
    success:True
}

```





###Description:

This API replaces current list of waypoints on autopilot with new list passed.

###Parameters:
    
    Following parameters are applicable for onboard C++ and Python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
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
* Name: /namespace/navigation/waypoint_set</br>
* Service Type: WaypointSet

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ````POST http://<ip>/ros/<namespace>/navigation/waypoint_set````
* JSON Request:
{
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
}
* JSON Response:
{
    success: Boolean
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/navigation/waypoint_set'</br>
* serviceType: 'core_api/WaypointSet'


### API usage information:
Note: You can either set body_frame or relative flag. If both are set, body_frame takes precedence.

Tip: Asynchronous mode - The API call would return as soon as the command has been sent to the autopilot, irrespective of whether the vehicle has reached the given setpoint or not.

Tip: Synchronous mode - The API call would wait for the function to return, which happens when either the position setpoint is reached or timeout=30secs is over.

