# Navigation APIs

## Arm



> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/arm
ROS-Service Type: core_api/Arm, below is its description

#Request : NULL

#Response : return success=true if command is successful
bool success
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition: int Navigation::arm(void)

Arguments:  None

Returns:    returns 0 if the command is successfully sent to the vehicle
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.api.navigation

Function: arm()
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/arm()
call srv: NULL
response srv: bool success
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/arm()
call srv: NULL
response srv: bool success

```

```javascript--REST
This is a REST call for the API to arm the 
FlytOS running device. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/arm'

JSON Response:
{   success: Boolean, }

```

```javascript--Websocket
This is a Websocket call for the API to arm the 
FlytOS running device. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/navigation/arm',
serviceType: 'core_api/Arm'

Request:
{}

Response:
{   success: Boolean, }


```


> Example

```shell
rosservice call /flytpod/navigation/arm "{}"    
```

```cpp
#include <core_script_bridge/navigation_bridge.h>

Navigation nav;
if(!nav.arm())
    cout<<"System ARMED";
else
    cout<<"Failed to ARM system";
```

```python
# create flyt_python navigation class instance
from flyt_python import api
drone = api.navigation()
# wait for interface to initialize
time.sleep(3.0)

drone.arm()
```

```cpp--ros
#include <core_api/Arm.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::Arm>("navigation/arm");
core_api::Arm srv;
client.call(srv);
success = srv.response.success;
```

```python--ros
from core_api.srv import *

def arm()
    rospy.wait_for_service(/<namespace>/navigation/arm')
    try:
        handle = rospy.ServiceProxy(/<namespace>/navigation/arm', Arm)
        resp = handle()
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("service call failed %s", e)

```

```javascript--REST

$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/navigation/arm",  
    success: function(data){
           console.log(data.success);
    }
};

```

```javascript--Websocket
var arm = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/arm',
    serviceType : 'core_api/Arm'
});

var request = new ROSLIB.ServiceRequest({});

arm.callService(request, function(result) {
    console.log('Result for service call on '
      + arm.name
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
This API arms the motors. If arm fails then check debug messages for arming errors. Likely reasons are unclaibrated sensors, misconfiguration.

###Parameters:
    
    Following parameters are applicable for onboard C++ and Python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments: None
    
    Output:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful

### ROS endpoint:
Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy api definition for message structure. 

* Type: Ros Service</br> 
* Name: /\<namespace\>/navigation/arm</br>
* Service Type: core_api/Arm

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ````GET http://<ip>/ros/<namespace>/navigation/arm````
* JSON Response:
{
    success: Boolean
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/\<namespace\>/navigation/arm'</br>
* serviceType: 'core_api/Arm'


### API usage information:

* ARM API will only work when device is in OFFBOARD / API CTL mode.
* All navigation setpoint API's except take_off require that drone is armed. So before calling any setpoint / waypoint APIs, drone should be armed.
* It is safer to use take_off command instead of arm command. 