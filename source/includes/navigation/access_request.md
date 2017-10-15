# Navigation APIs

## Access Request



> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/access_request
ROS-Service Type: core_api/AccessRequest, below is its description

#Request : 
bool enable_access

#Response : return success=true if command is successful
bool success
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition: int Navigation::access_request(bool enable_access)

Arguments:  enable_access: Set it to True to enable API access

Returns:    returns 0 if the command is successfully sent to the vehicle
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.api.navigation

Function: access_request(enable_access)
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/access_request
call srv:
    :bool enable_access
response srv: 
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/access_request
call srv:
    :bool enable_access
response srv: 
    :bool success
    :string message

```

```javascript--REST
This is a REST call to enable API control over drone. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/access_request'

JSON Request:
{
    enable_access: Boolean
}

JSON Response:
{   success: Boolean,
    message: String, }

```

```javascript--Websocket
This is a Websocket call to enable API control over drone. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/navigation/access_request',
serviceType: 'core_api/AccessRequest'

Request:
{   enable_access: Boolean}

Response:
{   success: Boolean,
    message: String, }


```


> Example

```shell
rosservice call /flytos/navigation/access_request "{enable_access: true}"    
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
if(!nav.access_request())
    cout<<"API access enabled";
else
    cout<<"Failed to enable API access";
```

```python
# create flyt_python navigation class instance
from flyt_python import api
drone = api.navigation()
# wait for interface to initialize
time.sleep(3.0)

drone.access_request()
```

```cpp--ros
#include <core_api/AccessRequest.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::AccessRequest>("/<namespace>/navigation/access_request");
core_api::AccessRequest srv;
srv.request.enable_access=true;
client.call(srv);
bool success = srv.response.success;
std::string message = srv.response.message;
```

```python--ros
import rospy
from core_api.srv import *

def access_request(enable_access)
    rospy.wait_for_service('/<namespace>/navigation/access_request')
    try:
        handle = rospy.ServiceProxy('/<namespace>/navigation/access_request', AccessRequest)
        resp = handle(enable_access=enable_access)
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("service call failed %s", e)
```

```javascript--REST

var  msgdata={};
msgdata["enable_access"]=true;

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/navigation/access_request",  
    success: function(data){
           console.log(data.success);
           console.log(data.message);
    }
};

```

```javascript--Websocket
var access_request = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/access_request',
    serviceType : 'core_api/AccessRequest'
});

var request = new ROSLIB.ServiceRequest({
    enable_access = true
});

access_request.callService(request, function(result) {
    console.log('Result for service call on '
      + access_request.name
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
0
```

```python
True
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
This API enables API control over drone. Sending vehicle to GUIDED/OFFBOARD mode via RC automatically enables API control, and likewise sending vehicle to RC modes such as MANUAL/STABILIZE/ALTCTL/ALT_HOLD/POSCTL/POSHOLD/LOITER disables API control. 

If this API is called with enable_access argument set to 'true', vehicle's mode is shifted to GUIDED/OFFBOARD mode internally.

###Parameters:
    
    Following parameters are applicable for onboard C++ and Python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments: None
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    enable_access | bool | Set this to true to enable API access. Setting this to false won't have any effect, this feature would be added later.

    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### ROS endpoint:
Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: Ros Service</br> 
* Name: /\<namespace\>/navigation/access_request</br>
* Service Type: core_api/AccessRequest

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ``POST http://<ip>/ros/<namespace>/navigation/access_request``
* JSON Request:
{
    enable_access: Boolean
}
* JSON Response:
{
    success: Boolean
    message: String
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/\<namespace\>/navigation/access_request'</br>
* serviceType: 'core_api/AccessRequest'


### API usage information:

* access_request API MUST be called if you don't have any RC hooked with the vehicle.
* It is safer to configure RC to communicate with the drone and send the vehicle to GUIDED/OFFBOARD mode instead of calling access_request command. 
* All navigation API's except 'disarm' requires that FlytOS's access to drone has been enabled. So before calling any setpoint / waypoint APIs, make sure to call this API, or send vehicle to GUIDED/OFFBOARD mode via RC.
