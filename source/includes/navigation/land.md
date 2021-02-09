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
string message
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
Name: /<namespace>/navigation/land
call srv: 
    async=false
response srv: 
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/land
call srv: 
    async=False
response srv: 
    :bool success
    :string message

```

```javascript--REST
This is a REST call for the API to land. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/land'

JSON Response:
{   
    success: Boolean,
    message: String
}

```

```javascript--Websocket
This is a Websocket call for the API to land. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/navigation/land',
serviceType: 'core_api/Land'

Request:
{  }

Response:
{   
    success: Boolean,
    message: String
}


```
```python--flyt_python

# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.flyt_python.DroneApiConnector

Function: land(async=False)

```



> Example

```shell
rosservice call /flytos/navigation/land "async=true" 
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
bool success = srv.response.success;
std::string message = srv.response.message;
```

```python--ros
import rospy
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
           console.log(data.message);
    }
});

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
    
drone.land(async=False)
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

```python--flyt_python
{
    success: True, 
    message: message

}
```

### Description:

Land vehicle at current position. Check API usage section below before using this API.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    async | bool | If true, asynchronous mode is set
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### API usage information:

This API will land the vehicle at current location. 

* This API can be used only in GUIDED or OFFBOARD or API|POSCTL mode.
* If any other navigation API is called during landing, then land call be overridden by that API call. 
* Automatic land flow can be configured with following parameters.
  * LNDMC_Z_VEL_MAX : Maximum velocity in vertical direction when landing (ideal value 0.8 m/s to 1.5 m/s)
  * LNDMC_XY_VEL_MAX: Maximum velocity in horizontal direction when landing (ideal value 1 m/s to 2 m/s)
  * MPC_LAND_SPEED: Landing velocity (ideal value 0.8 m/s)
* To disarm vehicle automatically after landing following parameter can be configured.
  * COM_DISARM_LAND:: 0 : disabled, n (integer between 1 to 20 inclusive) : enabled with n seconds timeout before disarming after landed. 
  * If this feature is enabled motors will disarm automatically even in cases where vehicle was armed but not flown. So for most scenarios value 5 should be fine. 

### ROS endpoint:

Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs.
Take a look at roscpp and rospy API definition for message structure. 

* Type: `Ros Service`
* Name: `/<namespace>/navigation/land`
* Service Type: `core_api/Land`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port **80**. RESTful APIs can be called from remote platform of your choice.

* URL: `GET http://<ip>/ros/<namespace>/navigation/land`
* JSON Response:
`{
    success: Boolean
    message: String
}`


### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 

Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace>/navigation/land`
* serviceType: `core_api/Land`

