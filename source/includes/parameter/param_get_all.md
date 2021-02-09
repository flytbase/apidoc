## Parameter Get All

> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/param/param_get_all
ROS-Service Type: core_api/ParamGetAll, below is its description

#Request : fresh pull true or false
bool fresh_pull

#Response: ParamInfo list of all parameters
core_api/ParamInfo[] param_list
#Response : success=true if command is successful.  
bool success
#Response : error message, if any
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition: bool Param::param_get_all(std::vector<core_api::ParamInfo> &param_list, bool fresh_pull)

Arguments:
    fresh_pull: Whether to fresh pull from autopilot or not
    param_list: Variable to store parameter list
Returns:
    returns parameter info list in param_list 
    returns 0 if the command is successful
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.
Class: flyt_python.API.navigation

Function: param_get_all()
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/param/param_get_all
call srv:
    :bool fresh_pull
response srv:
    :core_api/ParamInfo[] param_list
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/param/param_get_all
call srv:
    :bool fresh_pull
response srv:
    :core_api/ParamInfo[] param_list
    :bool success
    :string message

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/param/param_get_all'

JSON Response:
{   success: Boolean,
    message: String,
    param_list: [{ param_id: [String],
        param_value: [String]},{},{},...]
}

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/param/param_get_all',
serviceType: 'core_api/ParamGetAll'

Response:
{   success: Boolean,
    message: String,
    param_list: [{ param_id: [String],
        param_value: [String]},{},{},...] }


```
```python--flyt_python

# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.flyt_python.DroneApiConnector

Function: get_all_params()

```

> Example

```shell
rosservice call /flytos/param/param_get_all "fresh_pull: false" 
```

```cpp
#include <cpp_api/param_bridge.h>

Param param;
bool fresh_pull = false;
core_api/ParamInfo[] param_list;

param.param_get_all(param_list, fresh_pull);
std::cout << "Parameter list: " << param_list << std::endl;
```

```python
# create flyt_python navigation class instance
from flyt_python import API
drone = API.navigation()
# wait for interface to initialize
time.sleep(3.0)

#Get all parameter
drone.param_get_all()
```

```cpp--ros
// Please refer to Roscpp documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c%2B%2B)
```

```python--ros

# Please refer to Rospy documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)

```

```javascript--REST

$.ajax({
    type: "GET",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/param/param_get_all",  
    success: function(data){
           console.log(data.param_list);
    }
});

```

```javascript--Websocket
var paramGetAll = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/param/param_get_all',
    serviceType : 'core_api/ParamGetAll'
});

var request = new ROSLIB.ServiceRequest({});

paramGetAll.callService(request, function(result) {
    console.log('Result for service call on '
      + ParamGetAll.name
      + ': '
      + result.param_list);
});
```
```python--flyt_python 
from flyt_python.flyt_python import DroneApiConnector
token = ''                      # Personal Access Token
vehicle_id = ''                 # Vehicle ID

#create an instance of class DroneApiConnector
drone = DroneApiConnector(token,vehicle_id,ip_address='localhost' wait_for_drone_response =True)
drone.connect()
    
drone.get_all_params()

drone.disconnect()
```

> Example response

```shell
    success:True,
    param_list:[{},{},{},....]
```

```cpp
Function returns 0
param_list is populated with all the received parameters
```

```python
{'param_list': [{'param_value': '0.200000', 'param_id': 'ATT_VIBE_THRESH'}, {'param_value': '15.391030', 'param_id': 'BAT_A_PER_V'}], 'message': 'Received 2 parameters', 'success': True}

param_list (list): list of dictionary, each dictionary consists of param_value and param_id
message (String): Contains error/success message
success (bool): true if action successful
```

```cpp--ros
```

```python--ros
```

```javascript--REST
{
    success:True,
    param_list:[{},{},{},....]
}

```

```javascript--Websocket
{
    success:True,
    param_list:[{},{},{},....]    
}

```

```python--flyt_python
{
    success: True, 
    message: message,
    param_id: param_id,
    param_info: param_info
}
```

### Description:

This API gets all the parameters available in FlytOS with their values.

###Parameters:
    
    Following parameters are applicable in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message
    param_id | string | Name of the parameter
    param_value | string | value of the parameter

### ROS endpoint:

APIs in FlytOS are derived from/wrapped around the core services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: `Ros Service`
* Name: `/<namespace>/param/param_get_all`
* Service Type: `ParamGetAll`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port **80**. RESTful APIs can be called from remote platform of your choice.

* URL: `POST http://<ip>/ros/<namespace>/param/param_get_all`
* JSON Response:
`{
    success: Boolean,
    message: String,
    param_list:[{
        param_id: String,
        param_value: String
    },{},{},....]
}`

### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 

Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace>/param/param_get_all`
* serviceType: `core_api/ParamGetAll`