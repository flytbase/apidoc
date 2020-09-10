## Collision Avoidance

> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

ROS-Service Name: /<namespace>/payload/collision_avoidance_configure
ROS-Service Type: core_api/CollisionAvoidanceConfigure, below is its description

#Request : expects enable_avoidance to enable/disable collision avoidance
#Response : return success=true if command is successfully sent

bool enable_avoidance
---
bool success
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Not Implemented
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.
Not Implemented
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/payload/collision_avoidance_configure
call srv:
    :bool enable_avoidance
response srv: 
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/payload/collision_avoidance_configure
call srv:
    :bool enable_avoidance
response srv: 
    :bool success
    :string message
```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/payload/collision_avoidance_configure'

JSON Request:
{
    enable_avoidance: Boolean
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

name: '/<namespace>/payload/collision_avoidance_configure',
serviceType: 'core_api/CollisionAvoidanceConfigure'

Request:
{
    enable_avoidance: Boolean
}

Response:
{
    success: Boolean,
    message: String
}
```

> Example

```shell
rosservice call /<namespace>/payload/collision_avoidance_configure "enable_avoidance: false"
```

```cpp
Not Implemented
```

```python
Not Implemented
```

```cpp--ros
Not Implemented
```

```python--ros
Not Implemented
```

```javascript--REST
var  msgdata={};
msgdata["enable_avoidance"]=true;

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/payload/collision_avoidance_configure",  
    success: function(data){
           console.log(data.success);
           console.log(data.message);
    }
});
```

```javascript--Websocket
var collision_avoidance_configure = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/payload/collision_avoidance_configure',
    serviceType : 'core_api/CollisionAvoidanceConfigure'
});

var request = new ROSLIB.ServiceRequest({
    enable_avoidance: true
});

collision_avoidance_configure.callService(request, function(result) {
    console.log('Result for service call on '
      + gimbalSet.name
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
Not Implemented
```

```python
Not Implemented
```

```cpp--ros
Not Implemented
```

```python--ros
Not Implemented
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

This API enables/disables collision avoidance system of the DJI drone.

<aside class="warning">
    This API will **ONLY** work with FlytOS mobile app.
</aside>

### Parameters:
    
    Following parameters are applicable for onboard C++ and Python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    enable_avoidance | boolean | set to True to enable collision avoidance
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### ROS endpoint:

Payload APIs in FlytOS are derived from / wrapped around the core services available in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: `Ros Service`
* Name: `/<namespace>/payload/collision_avoidance_configure`
* Service Type: `core_api/CollisionAvoidanceConfigure`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port **80**. RESTful APIs can be called from remote platform of your choice.

* URL: `POST http://<ip>/ros/<namespace>/payload/collision_avoidance_configure`
* JSON Request:
`{
    enable_avoidance: Boolean
}`
* JSON Response:
`{
    success: Boolean,
    message: String
}`

### Websocket endpoint:

Websocket APIs can be called from javascript using [roslibjs library](https://github.com/RobotWebTools/roslibjs).

Java websocket clients are supported using [rosjava](http://wiki.ros.org/rosjava).

* name: `/<namespace>/payload/collision_avoidance_configure`
* serviceType: `core_api/CollisionAvoidanceConfigure`
