## Disarm



> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/disarm
ROS-Service Type: core_api/Disarm, below is its description

#Request : NULL

#Response : return success=true if command is successful
bool success
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition: int Navigation::disarm(void)

Arguments: None

Returns:    returns 0 if the command is successfully sent to the vehicle
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.api.navigation

Function: disarm():
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/disarm()
call srv: NULL
response srv: bool success
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/disarm()
call srv: NULL
response srv: bool success

```

```javascript--REST
This is a REST call for the API to disarm the 
FlytOS running device. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/disarm'

JSON Response:
{   success: Boolean, }

```

```javascript--Websocket
This is a Websocket call for the API to disarm the 
FlytOS running device. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/navigation/disarm',
serviceType: 'core_api/Disarm'

Request:
{}

Response:
{   success: Boolean, }


```


> Example

```shell
rosservice call /flytpod/navigation/disarm "{}"    
```

```cpp
#include <core_script_bridge/navigation_bridge.h>

Navigation nav;
if(!nav.disarm())
    cout<<"System DIARMED";
else
    cout<<"Failed to DISARM system";
```

```python
# create flyt_python navigation class instance
from flyt_python import api
drone = api.navigation()
# wait for interface to initialize
time.sleep(3.0)

drone.disarm()
```

```cpp--ros
#include <core_api/Disarm.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::Disarm>("navigation/disarm");
core_api::Disarm srv;
client.call(srv);
success = srv.response.success;
```

```python--ros
from core_api.srv import *

def disarm():
    rospy.wait_for_service('namespace/navigation/disarm')
    try:
        handle = rospy.ServiceProxy('namespace/navigation/disarm', Disarm)
        resp = handle()
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("service call failed %s", e)

```

```javascript--REST

$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/navigation/disarm",  
    success: function(data){
           console.log(data.success);
    }
};

```

```javascript--Websocket
var disarm = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/disarm',
    serviceType : 'core_api/Disarm'
});

var request = new ROSLIB.ServiceRequest({});

disarm.callService(request, function(result) {
    console.log('Result for service call on '
      + disarm.name
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
This API disarms the motors. Read API desciption below before you use it. Calling this API during flight will cause the motors to stall and may result in crash.

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
* Name: /\<namespace\>/navigation/disarm</br>
* Service Type: core_api/Disarm

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ````GET http://<ip>/ros/<namespace>/navigation/disarm````
* JSON Response:
{
    success: Boolean
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/\<namespace\>/navigation/disarm'</br>
* serviceType: 'core_api/Disarm'


### API usage information:

* This API will work regardless of what flight mode vehicle is in.
* Make sure that drone is on ground before disarming. If this API is called during flight, motors will stop instantly causing the drone to crash.
* To confirm whether vehicle is grounded / landed subscribe to following topic. (/global_namespace/mavros/extended_state), parameter name :  landed_state, value: 1 --> ground,  2 --> air/flying.  
* If land API is used then the vehicle will automatically disrm after some time. 
* Land API with auto diarm on landing feature is preferred over calling disarm API specifically.
* To configure auto disarm on landing set following parameters. 
  * COM_DISARM_LAND:: 0 : disabled, n (integer between 1 to 20 inculsive) : enabled with n seconds timeout before disarming after landed. 
  * If this feature is enabled motors will disarm automatically even in cases where vehicle was armed but not flown. So for most scenarios value 5 should be fine. 