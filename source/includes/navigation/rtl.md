## RTL


> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/rtl
ROS-Service Type: core_api/RTL, below is its description

#Request : NULL

#Response : returns success=true if RTL is activated
bool success
string message
```

```cpp
// CPP API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from cpp.

Function Definition:    int Navigation::rtl()

Arguments: None

Returns:    0 if the rtl command is successfully sent to the vehicle, else returns 1.
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from python.

Class: flyt_python.API.navigation

Function: rtl()
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/rtl
call srv: NULL
response srv: 
    :bool success
    :string message
```

```javascript--REST
This is a REST call for the API to transition the vehicle to RTL mode. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/rtl'

JSON Response:
{   success: Boolean,
    message: String, }
```

```javascript--Websocket
This is a Websocket call for the API to transition the vehicle to RTL mode. Make sure you 
initialise the websocket using websocket initialising 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/navigation/rtl',
serviceType: 'core_api/RTL'

Request:
{  }

Response:
{   success: Boolean,
    message: String, }
```


> Example

```shell
rosservice call /flytpod/navigation/rtl "{}" 
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
nav.rtl();
```

```python
# create flyt_python navigation class instance
from flyt_python import API
drone = API.navigation()
# wait for interface to initialize
time.sleep(3.0)

#trigger RTL mode 
drone.rtl() 
```

```cpp--ros
#include <core_api/RTL.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::RTL>("/<namespace>/navigation/rtl");
core_api::RTL srv;

client.call(srv);
bool success = srv.response.success;
std::string message = srv.response.message;
```

```python--ros
import rospy
from core_api.srv import *

def rtl():
    rospy.wait_for_service('/<namespace>/navigation/rtl')
    try:
        handle = rospy.ServiceProxy('/<namespace>/navigation/rtl', RTL)
        resp = handle()
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("service call failed %s", e)
```

```javascript--REST
$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/navigation/rtl",  
    success: function(data){
           console.log(data.success);
    }
};
```

```javascript--Websocket
var rtl = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/rtl',
    serviceType : 'core_api/RTL'
});

var request = new ROSLIB.ServiceRequest({});

rtl.callService(request, function(result) {
    console.log('Result for service call on '
      + rtl.name
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
{'message': '[INFO] RTL Triggered by FlytAPI', 'success': True}

message (string): Contains error message
success (bool): true if action successful
```

```cpp--ros
success: True
```

```python--ros
Success: True
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

Trigger RTL mode transition of the vehicle. Check API usage section below before using this API.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments: None
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### ROS endpoint:
Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: Ros Service</br> 
* Name: /\<namespace\>/navigation/rtl</br>
* Service Type: core_api/RTL

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ``GET http://<ip>/ros/<namespace>/navigation/rtl``
* JSON Response:
{
    success: Boolean
    message: String
}


### Websocket endpoint:
Websocket APIs can be called from javascript using [roslibjs library](https://github.com/RobotWebTools/roslibjs).
Java websocket clients are supported using [rosjava](http://wiki.ros.org/rosjava).

* name: '/\<namespace\>/navigation/rtl'</br>
* serviceType: 'core_api/RTL'


### API usage information:

This API will transition the vehicle to RTL mode.

* This API can be used only in GUIDED or OFFBOARD or API|POSCTL mode.
* If any other navigation API is called when vehicle is in RTL mode, then RTL call be overridden by that API call.
* Make sure to configure the following parameters, before triggering this mode.
  * RTL_RETURN_ALT : Altitude to fly back in RTL in meters.
	* RTL_DESCEND_ALT : RTL Loiter altitude. Vehicle stays at this altitude above home position and starts to land if autolanding is allowed.
  * RTL_LAND_DELAY : Delay after descend before landing in RTL mode. If set to -1 the system will not land but loiter at RTL_DESCEND_ALT. 
  * RTL_MIN_DIST : If the system is horizontally closer than this distance to home it will land straight on home instead of raising to the return altitude first. 

