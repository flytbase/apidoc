## Parameter Get


> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

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

Class: flyt_python.API.navigation

Function: param_get(param_id)
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/param/param_get
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
Name: /<namespace>/param/param_get
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
    message: String,
    param_info:{ param_value: String } }

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/param/param_get',
serviceType: 'core_api/ParamGet'

Request:
{   param_id: String }

Response:
{   success: Boolean,
    message: String,
    param_info:{ param_value: String } }


```


> Example

```shell
rosservice call /flytpod/param/param_get "param_id: ''"
```

```cpp
#include <cpp_api/param_bridge.h>

Param param;
std::string param_id = "RTL_ALT"; 
std::string param_value;

param.param_get(param_id, param_value);
std::cout << "Parameter value: " << param_value << std::endl;
```

```python
# create flyt_python navigation class instance
from flyt_python import API
drone = API.navigation()
# wait for interface to initialize
time.sleep(3.0)

#Get value of a specific parameter
drone.param_get('walk')
```

```cpp--ros
// Please refer to Roscpp documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c%2B%2B)
```

```python--ros

# Please refer to Rospy documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)

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
String - the value of the param
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
This API gets the value of a particular parameter specified.

###Parameters:
    
    Following parameters are applicable in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    param_id | string | Name of the parameter

    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message
    param_value | string | value of the parameter

### ROS endpoint:
APIs in FlytOS are derived from / wrapped around the core  services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: Ros Service</br> 
* Name: /\<namespace\>/param/param_get</br>
* Service Type: ParamGet

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ``POST http://<ip>/ros/<namespace>/param/param_get``
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

* name: '/\<namespace\>/param/param_get'</br>
* serviceType: 'core_api/ParamGet'


<!-- ### API usage information:
Note: You can either set body_frame or relative flag. If both are set, body_frame takes precedence.

Tip: Asynchronous mode - The API call would return as soon as the command has been sent to the autopilot, irrespective of whether the vehicle has reached the given setpoint or not.

Tip: Synchronous mode - The API call would wait for the function to return, which happens when either the position setpoint is reached or timeout=30secs is over.
 -->
