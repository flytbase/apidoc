# Namespace

> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login. 

ROS-Service Name: /get_global_namespace
ROS-Service Type: core_api/ParamGetGlobalNamespace, below is its description

#Request : None

#Response : Paramter info
core_api/ParamInfo param_info
#Response : success=true if parameter get was successfull.
bool success
#Response : Returns error message/success message if any.
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.
std::string global_namespace

//No API call required. Global namespace already available in private variable: std::string global_namespace
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.api.navigation
Function Definition: get_global_namespace()
Arguments: None
return: string
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /get_global_namespace()
call srv: NULL
response srv: 
    :bool success
    :core_api/ParamInfo param_info
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /get_global_namespce()
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
		param_info:{
			param_value: String
		}
	}

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/get_global_namespace',
serviceType: 'core_api/ParamGetGlobalNamespace'

Response:
{   success: Boolean,
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
//No API call required. Global namespace already available in private variable: std::string global_namespace
```

```python
# create flyt_python navigation class instance

from flyt_python import api
drone = api.navigation()
time.sleep(3.0)
namespace = drone.get_global_namespace()

```

```cpp--ros
#include <core_api/ParamGetGlobalNamespace.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::ParamGetGlobalNamespace>("navigation/get_global_namespace");
core_api::ParamGetGlobalNamespace srv;
client.call(srv);
std::string param_id = srv.response.param_info.param_id;
std::string param_value = srv.response.param_info.param_value;
bool success = srv.response.success;
std::string = srv.response.message;
```

```python--ros
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


> Example

```shell
param_info: 
  param_id: global_namespace
  param_value: flytpod
success: True
message: Parameter Get Global Namespace Successful  flytpod
```

```cpp
//No API call required. Global namespace already available in private variable: std::string global_namespace
```

```python
flytpod
```

```cpp--ros
std::string param_id = srv.response.param_info.param_id;
std::string param_value = srv.response.param_info.param_value;
bool success = srv.response.success;
std::string = srv.response.message;
```

```python--ros
Response object with following structure.
param_info: 
  param_id: global_namespace
  param_value: flytpod
success: True
message: Parameter Get Global Namespace Successful	flytpod
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

This API returns the global namespace.

###Parameters:
    
    Following parameters are applicable for onboard C++ and Python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Output:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    namespace | string | global namespace name

### ROS endpoint:
Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy api definition for message structure. 

* Type: Ros Service</br> 
* Name: /get_global_namespace</br>
* Service Type: ParamGetGlobalNamespace

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ````GET http://<ip>/ros/get_global_namespace````
* JSON Response:
{
    success: Boolean,
    param_info:{param_value: String}
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/get_global_namespace'</br>
* serviceType: 'core_api/ParamGetGlobalNamespace'


### API usage information:

* Global namespce is required when calling onboard/remote APIs except python APIs.
* All the core nodes of FlytOS run inside this namespace.
* Make sure that your custom ROS packages are not launched inside this namespace.
