## Land


> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/land
ROS-Service Type: core_api/Land, below is its description

#Request : expects async variable to be set/reset
bool async

#Response : return success=true if Land command sent successfully to autopilot
bool success
```

```cpp
// CPP API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from cpp.

Function Definition:    int Navigation::land(bool async = false)

Arguments:
    async: If true, asynchronous mode is set

Returns:    0 if the land command is successfully sent to the vehicle, else returns 1.
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from python.

Class: flyt_python.api.navigation

Function: land(async=False):
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/land()
call srv: 
    async=false
response srv: bool success
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/land()
call srv: 
    async=False
response srv: bool success

```

```javascript--REST
This is a REST call for the API to land. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/land'

JSON Response:
{   success: Boolean, }

```

```javascript--Websocket
This is a Websocket call for the API to land. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/navigation/land',
serviceType: 'core_api/Land'

Request:
{  }

Response:
{   success: Boolean, }


```


> Example

```shell
rosservice call /flytpod/navigation/land "async=true" 
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
nav.land(true);
```

```python
# create flyt_python navigation class instance
from flyt_python import api
drone = api.navigation()
# wait for interface to initialize
time.sleep(3.0)

# land at current location. Return after landed
drone.land(async=False)

# land at current location. Function returns immediately and land action finishes asynchronously.  
drone.land(async=True)
```

```cpp--ros
#include <core_api/Land.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::Land>("/<namespace>/navigation/land");
core_api::Land srv;

srv.request.async = true;
client.call(srv);
success = srv.response.success;
```

```python--ros
from core_api.srv import *

def land(async= False):
    rospy.wait_for_service('/<namespace>/navigation/land')
    try:
        handle = rospy.ServiceProxy('/<namespace>/navigation/land', Land)
        resp = handle(async)
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("service call failed %s", e)

```

```javascript--REST

$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/navigation/land",  
    success: function(data){
           console.log(data.success);
    }
};

```

```javascript--Websocket
var land = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/land',
    serviceType : 'core_api/Land'
});

var request = new ROSLIB.ServiceRequest({});

land.callService(request, function(result) {
    console.log('Result for service call on '
      + land.name
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

Land vehicle at current position. Check API usage section below before using this API.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTFul, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    async | bool | If true, asynchronous mode is set
    
    Output:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful

### ROS endpoint:
Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy api definition for message structure. 

* Type: Ros Service</br> 
* Name: /\<namespace\>/navigation/land</br>
* Service Type: core_api/Land

### RESTFul endpoint:
FlytOS hosts a RESTFul server which listens on port 80. RESTFul APIs can be called from remote platform of your choice.

* URL: ````GET http://<ip>/ros/<namespace>/navigation/land````
* JSON Response:
{
    success: Boolean
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/\<namespace\>/navigation/land'</br>
* serviceType: 'core_api/Land'


### API usage information:

This API will land the vehicle at current location. 

* This API can be used only in OFFBOARD/API_CTL mode.
* If in synchronous mode and another navigation API is called then land call be overridden by that API call. 
* Automatic land flow can be configured with following parameters.
  * LNDMC_Z_VEL_MAX : Maximum velocity in vertical direction when landing (ideal value 0.8 m/s to 1.5 m/s)
  * LNDMC_XY_VEL_MAX: Maximum velocity in horizontal direction when landing (ideal value 1 m/s to 2 m/s)
  * MPC_LAND_SPEED: Landing velocity (ideal value 0.8 m/s)
* To disarm vehicle automatically after landing following parameter can be configured.
  * COM_DISARM_LAND:: 0 : disabled, n (integer between 1 to 20 inclusive) : enabled with n seconds timeout before disarming after landed. 
  * If this feature is enabled motors will disarm automatically even in cases where vehicle was armed but not flown. So for most scenarios value 5 should be fine. 