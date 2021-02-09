## Parameter Delete

> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/param/param_delete
ROS-Service Type: core_api/ParamDelete, below is its description

#Request : Info of parameter to be deleted
string param_id

#Response : success=true if command is successful.  
bool success
#Response : error message, if any
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition: bool Param::param_delete(std::string param_id)

Arguments:
    param_id: ID of param to be deleted
Returns: returns 0 if the command is successful
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.API.navigation

Function: param_delete()
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/param/param_delete
call srv:
    :string param_id
response srv: 
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/param/param_delete
call srv:
    :string param_id
response srv: 
    :bool success
    :string message

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/param/param_delete'

JSON Request:
{   param_id: String }

JSON Response:
{   success: Boolean,
    message: String, }

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/param/param_delete',
serviceType: 'core_api/ParamDelete'

Request:
{   param_id: String }

Response:
{   success: Boolean,
    message: String, }
```

```python--flyt_python

# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.flyt_python.DroneApiConnector

Function: delete_params(self, param_id)

```


> Example

```shell
rosservice call /flytos/param/param_delete "param_id: ''" 
```

```cpp
#include <cpp_api/param_bridge.h>

Param param;
std::string param_id = "RTL_ALT"; 
param.param_delete(param_id);
```

```python
# create flyt_python navigation class instance
from flyt_python import API
drone = API.navigation()
# wait for interface to initialize
time.sleep(3.0)

#Get value of a specific parameter
drone.param_delete()
```

```cpp--ros
// Please refer to Roscpp documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c%2B%2B)
```

```python--ros

# Please refer to Rospy documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)

```

```javascript--REST
var  msgdata={};
msgdata["param_id"]='RTL_ALT;

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/param/param_delete",  
    success: function(data){
           console.log(data.success);
           console.log(data.message);
    }
});

```

```javascript--Websocket
var paramDelete = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/param/param_delete',
    serviceType : 'core_api/ParamDelete'
});

var request = new ROSLIB.ServiceRequest({
    param_id: String
});

paramDelete.callService(request, function(result) {
    console.log('Result for service call on '
      + paramDelete.name
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
    
drone.delete_params(param_id= String)

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
Bool - True, if action successful
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

This API deletes a parameter from FlytOS.

### Parameters:
    
    Following parameters are applicable in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    param_id | string | Name of the paramter to be deleted
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### ROS endpoint:

APIs in FlytOS are derived from / wrapped around the core  services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: `Ros Service`
* Name: `/<namespace>/param/param_delete`
* Service Type: `ParamDelete`

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port **80**. RESTful APIs can be called from remote platform of your choice.

* URL: `POST http://<ip>/ros/<namespace>/param/param_delete`
* JSON Request:
`{
    param_id: String
}`
* JSON Response:
`{
    success: Boolean,
    message: String
}`

### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 

Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace>/param/param_delete`
* serviceType: `core_api/ParamDelete`
