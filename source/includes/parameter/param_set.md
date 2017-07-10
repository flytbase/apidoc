# Parameter APIs

## Parameter Set


> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/param/param_set
ROS-Service Type: core_api/ParamSet, below is its description

#Request : Info of parameter to be set
core_api/ParamInfo param_info

#Response : success=true if command is successful.  
bool success
#Response : error message, if any
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition: bool Param::param_set(std::string param_id, std::string param_value)

Arguments:
    param_id: ID of param to be set
    param_value: Value of param to be set
Returns: returns 0 if the command is successfull
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.
Class: flyt_python.API.navigation

Function: param_set(param_id, param_value)
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/param/param_set
call srv:
    :core_api/ParamInfo param_info
    :bool success
response srv: bool success
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/param/param_set
call srv:
    :core_api/ParamInfo param_info
    :bool success
response srv: bool success

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/param/param_set'

JSON Request:
{   param_info:{
        param_id: String,
        param_value: String
}}

JSON Response:
{   success: Boolean, }

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/param/param_set',
serviceType: 'core_api/ParamSet'

Request:
{   param_info:{
        param_id: String,
        param_value: String }}

Response:
{   success: Boolean, }


```


> Example

```shell
rosservice call /flytpod/param/param_set "param_info:
  param_id: ''
  param_value: ''" 

```

```cpp
#include <cpp_api/param_bridge.h>

Param param;
std::string param_id = "RTL_ALT"; 

std::string param_value = "5";
param.param_set(param_id, param_value);
```

```python
# create flyt_python navigation class instance
from flyt_python import API
drone = API.navigation()
# wait for interface to initialize
time.sleep(3.0)

#Set parameter
drone.param_set('walk', 'on')
```

```cpp--ros
// Please refer to Roscpp documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c%2B%2B)
```

```python--ros

# Please refer to Rospy documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)

```

```javascript--REST
var  msgdata={};msgdata["param_info"]={};
msgdata.param_info["param_id"]="RTL_ALT";
msgdata.param_info["param_value"]="5.0";

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/param/param_set",  
    success: function(data){
           console.log(data.success);
    }
};

```

```javascript--Websocket
var paramSet = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/param/param_set',
    serviceType : 'core_api/ParamSet'
});

var request = new ROSLIB.ServiceRequest({
    param_info:{
        param_id: “RTL_ALT”,
        param_value: “5.0”
});

paramSet.callService(request, function(result) {
    console.log('Result for service call on '
      + paramSet.name
      + ': '
      + result.success);
});
```


> Example response

```shell
success: true
```

```cpp
0
```

```python
bool : true if action successful
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
This API sets the value of a desired parameter

###Parameters:
    
    Following parameters are applicable in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    param_id | string | Name of the parameter to be updated
    param_value | string | Value of the parameter to be set.
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### ROS endpoint:
APIs in FlytOS are derived from / wrapped around the core services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: Ros Service</br> 
* Name: /\<namespace\>/param/param_set</br>
* Service Type: ParamSet

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ``POST http://<ip>/ros/<namespace>/param/param_set``
* JSON Request:
{
    param_info:{
        param_id: [String],
        param_value: [String]
    }
}
* JSON Response:
{
    success: Boolean
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/\<namespace\>/param/param_set'</br>
* serviceType: 'core_api/ParamSet'


### API usage information:
Note: Make sure the parameter exists, which you are trying to update.



