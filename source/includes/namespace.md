# Namespace API

> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login. 

ROS-Service Name: /get_global_namespace
ROS-Service Type: core_api/ParamGetGlobalNamespace, below is its description

#Request : None

#Response : Paramter info
core_api/ParamInfo param_info
#Response : success=true if parameter get was successful.
bool success
#Response : Returns error message/success message if any.
string message
```

```cpp
// global namespace is not required for any CPP API
```

```python
# global namespace is not required for any Python API
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /get_global_namespace
call srv: NULL
response srv: 
    :bool success
    :core_api/ParamInfo param_info
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /get_global_namespace
response srv: ParamGetGlobalNamespace
```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: ' <ip>/ros/get_global_namespace'

JSON Response:
	{
		success: Boolean,
    message: String,
		param_info:{
			param_value: String
		}
	}
```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/get_global_namespace',
serviceType: 'core_api/ParamGetGlobalNamespace'

Response:
{   success: Boolean,
    message: String,
    param_info:{
            param_value: String
        }
}
```


> Example

```shell
rosservice call /get_global_namespace "{}"
```

```cpp
// global namespace is not required for any CPP API
```

```python
# global namespace is not required for any Python API
```

```cpp--ros
#include <core_api/ParamGetGlobalNamespace.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::ParamGetGlobalNamespace>("/get_global_namespace");
core_api::ParamGetGlobalNamespace srv;
client.call(srv);

std::string global_namespace = srv.response.param_info.param_value;

bool success = srv.response.success;
std::string message = srv.response.message;
```

```python--ros
import rospy
from core_api.srv import *

def get_global_namespace():
    rospy.wait_for_service('/get_global_namespace')
    try:
        res = rospy.ServiceProxy('/get_global_namespace', ParamGetGlobalNamespace)
        op = res()
        return str(op.param_info.param_value)
    except rospy.ServiceException, e:
        rospy.logerr("global namespace service not available", e)
        return None
```


```javascript--REST
	$.ajax({
	    type: "GET",
	    dataType: "json",
	    url: "http://<ip>/ros/get_global_namespace",   
	    success: function(data){
	        console.log(data.param_info.param_value);
	    }
	});
```

```javascript--Websocket
var namespace = new ROSLIB.Service({
    ros : ros,
    name : '/get_global_namespace',
    serviceType : 'core_api/ParamGetGlobalNamespace'
});

var request = new ROSLIB.ServiceRequest({});

namespace.callService(request, function(result) {
    console.log('Result for service call on '
      + namespace.name
      + ': '
      + result.param_info.param_value);
});
```


> Example response

```shell
param_info: 
  param_id: global_namespace
  param_value: flytpod
success: True
message: FlytOS namespace is  flytpod
```

```cpp
// global namespace is not required for any CPP API
```

```python
# global namespace is not required for any Python API
```

```cpp--ros
success : true
message : FlytOS namespace is  flytpod
```

```python--ros
success: True
message: FlytOS namespace is  flytpod
```


```javascript--REST
{
	success:True,
	param_info:{
		param_value:'flytpod'
	}
}
```

```javascript--Websocket
{
    success:True,
    param_info:{
        param_value:'flytpod'
    }
}
```





###Description:

This API returns the global namespace under which FlytOS's instance is running. For users using RESTful, Websocket or ROS APIs, calling this API is a **MUST**, as the value of this namespace is required to call other APIs. In the subsequent documentation, wherever *\<namespace\>* is mentioned in API call definition, it must be replaced by the output of this API call. Users writing their code in simple CPP and Python need not call this API.

### ROS endpoint:

* Service Type: `Ros Service`
* Name: `/get_global_namespace`
* Service Type: `ParamGetGlobalNamespace`

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: `http://<ip>/ros/get_global_namespace`
* Request Method: `GET`
* JSON Response: `{success: Boolean,message: String,param_info:{param_value: String}}`

### Websocket endpoint:
Websocket APIs can be called from javascript using [roslibjs library](https://github.com/RobotWebTools/roslibjs).
Java websocket clients are supported using [rosjava](http://wiki.ros.org/rosjava).

* name: `/get_global_namespace`
* serviceType: `core_api/ParamGetGlobalNamespace`


### API usage information:

* Global namespace is required when calling onboard/remote APIs except cpp/python APIs.
* All the core nodes of FlytOS run inside this namespace.
* Make sure that your custom ROS packages are not launched inside this namespace.
