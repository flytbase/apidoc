## Parameter Get


> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

ROS-Service Name: /<namespace>/param/param_get
ROS-Service Type: core_api/ParamGet, below is its description

#Request : Param id to get
string param_id

#Response: Param info of requested param
core_api/ParamInfo param_info
#Response : success=true if command is successful.  
bool success
#Response : error message, if any
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition: bool Param::param_get(std::string param_id, std::string &param_value)

Arguments:
    param_id: ID of param to be created
    param_value: Variable to store parameter value
Returns:
    returns parameter value in param_value 
    returns 0 if the command is successfull
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

NotImplemented
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/param/param_get()
call srv:
    :string param_id
response srv: 
    :core_api/ParamInfo param_info
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/param/param_get()
call srv:
    :string param_id
response srv: 
    :core_api/ParamInfo param_info
    :bool success
    :string message

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/param/param_get'

JSON Request:
{   param_id: String }

JSON Response:
{   success: Boolean,
    param_info:{ param_value: String } }

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/param/param_get',
serviceType: 'core_api/ParamGet'

Request:
{   param_id: String }

Response:
{   success: Boolean,
    param_info:{ param_value: String } }


```


> Example

```shell
rosservice call /flytpod/param/param_get "param_id: ''"
```

```cpp
#include <core_script_bridge/param_bridge.h>

Param param;
std::string param_id = "RTL_ALT"; 
std::string param_value;

param.param_get(param_id, param_value);
std::cout << "Parameter value: " << param_value << std::endl;
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
var  msgdata={};
msgdata["param_id"]="RTL_ALT";

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/param/param_get",  
    success: function(data){
           console.log(data.param_info.param_value);
    }
};

```

```javascript--Websocket
var paramGet = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/param/param_get',
    serviceType : 'core_api/ParamGet'
});

var request = new ROSLIB.ServiceRequest({
    param_id: 'RTL_ALT'
});

paramGet.callService(request, function(result) {
    console.log('Result for service call on '
      + paramGet.name
      + ': '
      + result.param_info.param_value);
});
```


> Example response

```shell
    success:True,
    param_info:{ param_value: '6.00'}
```

```cpp
param_value = 6
0
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
    success:True,
    param_info:{ param_value: '6.00'}
}

```

```javascript--Websocket
{
    success:True,
    param_info:{ param_value: '6.00'}
}

```





###Description:
This API sends local position setpoint command to the autopilot. Additionally, you can send yaw setpoint (yaw_valid flag must be set true) to the vehicle as well. Some abstract features have been added, such as tolerance/acceptance-radius, synchronous/asynchronous mode, sending setpoints relative to current position (relative flag must be set true), sending setpoints relative to current body frame (body_frame flag must be set true).
This command commands the vehicle to go to a specified location and hover. It overrides any previous mission being carried out and starts hovering.

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
* Name: /namespace/param/param_get</br>
* Service Type: ParamGet

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ````POST http://<ip>/ros/<namespace>/param/param_get````
* JSON Request:
{
    param_id: String
}
* JSON Response:
{
    success: Boolean
    param_info:{param_value: String}
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/param/param_get'</br>
* serviceType: 'core_api/ParamGet'


### API usage information:
Note: You can either set body_frame or relative flag. If both are set, body_frame takes precedence.

Tip: Asynchronous mode - The API call would return as soon as the command has been sent to the autopilot, irrespective of whether the vehicle has reached the given setpoint or not.

Tip: Synchronous mode - The API call would wait for the function to return, which happens when either the position setpoint is reached or timeout=30secs is over.

