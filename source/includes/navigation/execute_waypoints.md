## Execute Waypoints


> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/waypoint_execute
ROS-Service Type: core_api/waypointExecute, below is its description

#Request : Null

#Response : success = true if command sent successfully
bool success
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
Name: /<namespace>/navigation/waypoint_execute()
call srv: NULL
response srv: bool success
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/waypoint_execute()
call srv: NULL
response srv: bool success

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/waypoint_execute'

JSON Response:
{   success: Boolean, }

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/navigation/waypoint_execute',
serviceType: 'core_api/WaypointExecute'

Response:
{   success: Boolean, }


```


> Example

```shell
rosservice call /flytsim/navigation/waypoint_execute "{}"   
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
    url: "http://<ip>/ros/<namespace>/navigation/waypoint_execute",  
    success: function(data){
           console.log(data.success);
    }
};

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
      + result.success);
});
```


> Example response

```shell
success: true
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

Exectute / resume current list of waypoints.

###Parameters:
    
    Following parameters are applicable in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful

### ROS endpoint:
Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy api definition for message structure. 

* Type: Ros Service</br> 
* Name: /namespace/navigation/waypoint_execute</br>
* Service Type: WaypointExecute

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ````GET http://<ip>/ros/<namespace>/navigation/waypoint_execute````
* JSON Response:
{
    success: Boolean
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/navigation/waypoint_execute'</br>
* serviceType: 'core_api/WaypointExecute'


### API usage information:
Note: Make sure you have a list of waypoints already set using set_waypoints API before you give it execute_waypoint API call.

