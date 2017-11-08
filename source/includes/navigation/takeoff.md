## Take Off


> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/takeoff
ROS-Service Type: core_api/TakeOff, below is its description

# Request : expects take off altitude in metres
float32 takeoff_alt

# Response : returns success=true if takeoff altitude is reached
bool success
string message
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Function Definition:  int Navigation::take_off(float takeoff_alt = 5.0)

Arguments: 
    takeoff_alt: TakeOff Altitude in meters with default value of 5.0

Returns: 0 if the vehicle reaches takeoff_alt before timeout=30sec, else returns 1.
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.api.navigation

Function: take_off(self, takeoff_alt=5.0):
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/takeoff
call srv: 
    : int takeoff_alt
response srv: 
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/takeoff
Type: core_api/TakeOff
call srv: 
    : int takeoff_alt
response srv: 
    :bool success
    :string message

```

```javascript--REST
This is a REST call for the API to takeoff. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/take_off'

JSON Request:
{   takeoff_alt: Float }

JSON Response:
{   success: Boolean,
    message: String, }

```

```javascript--Websocket
This is a Websocket call for the API to takeoff. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/navigation/take_off',
serviceType: 'core_api/TakeOff'

Request:
{   takeoff_alt: Float }

Response:
{   success: Boolean,
    message: String, }


```

> Example

```shell
rosservice call /flytos/navigation/take_off "takeoff_alt: 3.0"
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
nav.take_off(3.0);
```

```python
# create flyt_python navigation class instance
from flyt_python import api
drone = api.navigation()
# wait for interface to initialize
time.sleep(3.0)

# takeoff over current location 
drone.take_off(6.0)
```

```cpp--ros
#include <core_api/TakeOff.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::TakeOff>("/<namespace>/navigation/takeoff");
core_api::TakeOff srv;

srv.request.takeoff_alt = 3.0;
client.call(srv);
bool success = srv.response.success;
std::string message = srv.response.message;
```

```python--ros
import rospy
from core_api.srv import *

def takeoff(height)
    rospy.wait_for_service('/<namespace>/navigation/take_off')
    try:
        handle = rospy.ServiceProxy('/<namespace>/navigation/take_off', TakeOff)
        resp = handle(takeoff_alt=height)
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("service call failed %s", e)

```

```javascript--REST
var  msgdata={};
msgdata["takeoff_alt"]=5.00;

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/navigation/take_off",  
    success: function(data){
           console.log(data.success);
           console.log(data.message);
    }
};

```

```javascript--Websocket
var takeoff = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/take_off',
    serviceType : 'core_api/TakeOff'
});

var request = new ROSLIB.ServiceRequest({
    takeoff_alt: 5.00
});

takeoff.callService(request, function(result) {
    console.log('Result for service call on '
      + takeoff.name
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

Takeoff and reach specified height from current location.

### Parameters:
    
    Following parameters are applicable for onboard C++ and Python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    takeoff_alt | float32 | takeoff to given height at current location. (minimum 1.5 meters)

    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### API usage information:

Takeoff to specified height from current height at current location.

* takeoff_alt value should be positive. 
* Irrespective of current altitude vechile will climb up by takeoff_alt meters from current location.
* Takeoff API will automatically arm the motors. 
* Takeoff API will work only in OFFBOARD/GUIDED/API|POSCTL mode.
* Minimum value of takeoff_alt argument is 1.5 meters.
* Takeoff API is always synchronous. 
* It is recommended not to send any other navigation commands while takeoff is under way.

### ROS endpoint:

Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: `Ros Service`
* Name: `/<namespace>/navigation/takeoff`
* Service Type: `core_api/TakeOff`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port **80**. RESTful APIs can be called from remote platform of your choice.

* URL: `POST http://<ip>/ros/<namespace>/navigation/take_off`
* JSON Request:
`{
    takeoff_alt: Float
}`
* JSON Response:
`{
    success: Boolean
    message: String
}`

### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace>/navigation/take_off`
* serviceType: `core_api/TakeOff`
