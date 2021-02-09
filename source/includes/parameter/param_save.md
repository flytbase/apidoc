## Parameter Save

> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/param/param_save
ROS-Service Type: core_api/ParamSave, below is its description

#Request : Null

#Response : success=true if command is successful.  
bool success
#Response : error message, if any
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition: bool Param::param_save(void)

Arguments: Null
Returns: returns 0 if the command is successful
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.API.navigation

Function: param_save()
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/param/param_save
call srv: Null
response srv: 
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/param/param_save
call srv: Null
response srv: 
    :bool success
    :string message

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/param/param_save'

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

name: '/<namespace>/param/param_save',
serviceType: 'core_api/ParamSave'

Response:
{   success: Boolean,
    message: String
}

```

```python--flyt_python

# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.flyt_python.DroneApiConnector

Function: save_params()

```


> Example

```shell
rosservice call /flytos/param/param_save "{}"
```

```cpp
#include <cpp_api/param_bridge.h>

Param param;

param.param_save()
```

```python
# create flyt_python navigation class instance
from flyt_python import API
drone = API.navigation()
# wait for interface to initialize
time.sleep(3.0)

#Get value of a specific parameter
drone.param_save()
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
    url: "http://<ip>/ros/<namespace>/param/param_save",  
    success: function(data){
           console.log(data.success);
           console.log(data.message);
    }
});

```

```javascript--Websocket
var paramSave = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/param/param_save',
    serviceType : 'core_api/ParamSave'
});

var request = new ROSLIB.ServiceRequest({});

paramSave.callService(request, function(result) {
    console.log('Result for service call on '
      + paramSave.name
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
    
drone.save_params()

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


###Description:

This API saves the parameters to a file which allows data retention on reboot of FlytOS running systems.

###Parameters:
    
    Following parameters are applicable in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### ROS endpoint:

APIs in FlytOS are derived from / wrapped around the core services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: `Ros Service`
* Name: `/<namespace>/param/param_save`
* Service Type: `ParamSave`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port **80**. RESTful APIs can be called from remote platform of your choice.

* URL: `GET http://<ip>/ros/<namespace>/param/param_save`
* JSON Response:
`{
    success: Boolean
    message: String
}`


### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 

Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace`>/param/param_save`
* serviceType: `core_api/ParamSave`