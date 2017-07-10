## Set Home


> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/navigation/set_home
ROS-Service Type: core_api/SetHome, below is its description

#Request : Expects home position to be set by specifying Latitude, Longitude and altitude
#Request: If set_current is true, the current location of craft is set as home position
float64 lat
float64 lon
float64 alt
bool set_current

#Response : success=true if service called successfully 
bool success
string message
```

```cpp
No CPP API available.
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.
Class: flyt_python.API.navigation

Function: set_home(lat, lon, alt, set_current=False)
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/set_home
call srv:
    :float64 lat
    :float64 lon
    :float64 alt
    :bool set_current
response srv: 
    :bool success
    :string message
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/set_home
call srv:
    :float64 lat
    :float64 lon
    :float64 alt
    :bool set_current
response srv: 
    :bool success
    :string message

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/navigation/set_home'

JSON Request:
{   lat: Float,
    lon: Float,
    alt: Float,
    set_current : Boolean }

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

name: '/<namespace>/navigation/set_home',
serviceType: 'core_api/SetHome'

Request:
{   lat: Float,
    lon: Float,
    alt: Float,
    set_current : Boolean }

Response:
{   success: Boolean,
    message: String, }


```


> Example

```shell
rosservice call /flytsim/navigation/set_home "{lat: 73.25564541, lon: 18.2165632, alt: 2.0, set_current: false}"  
```

```cpp
```

```python
# create flyt_python navigation class instance
from flyt_python import API
drone = API.navigation()
# wait for interface to initialize
time.sleep(3.0)

#set given location as home 
drone.set_home(10.2, 10.2, 10.2)

#set current location as home
drone.set_home(0, 0, 0, True)
```

```cpp--ros
#include <core_api/SetHome.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::SetHome>("/<namespace>/navigation/set_home");
core_api::SetHome srv;

srv.request.lat = 73.25564541;
srv.request.lon = 18.2165632;
srv.request.alt = 2.00;
srv.request.set_current = false;
client.call(srv);
bool success = srv.response.success;
std::string message = srv.response.message;
```

```python--ros

# Please refer to Rospy documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)

```

```javascript--REST
var  msgdata={};
msgdata["lat"]=73.25564541;
msgdata["lon"]=18.36155;
msgdata["alt"]=2.00;
msgdata["set_current"]=true;


$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/navigation/set_home",  
    success: function(data){
           console.log(data.success);
           console.log(data.message);
    }
};

```

```javascript--Websocket
var setHome = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/navigation/set_home',
    serviceType : 'core_api/SetHome'
});

var request = new ROSLIB.ServiceRequest({
    lat: 73.12516255,
    lon: 18.2165632,
    alt: 2.00,
    set_current : True 
});

setHome.callService(request, function(result) {
    console.log('Result for service call on '
      + setHome.name
      + ': '
      + result.success);
});
```


> Example response

```shell
success: true
```

```cpp
```

```python
{'message': '[INFO] Home Position successfully sent to vehicle', 'success': True}

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

Manually store a location as new home.

###Parameters:
    
    Following parameters are applicable for onboard C++ and Python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    lat,long,alt | float | Latitude, longitude and relative altitude
    set_current | boolean | if true uses current location and altitude of the device else uses the provided values.
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### ROS endpoint:
Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: Ros Service</br> 
* Name: /\<namespace\>/navigation/set_home</br>
* Service Type: SetHome

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ``POST http://<ip>/ros/<namespace>/navigation/set_home``
* JSON Request:
{
    lat: Float,
    lon: Float,
    alt: Float,
    set_current : Boolean 
}
* JSON Response:
{
    success: Boolean
    message: String
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/\<namespace\>/navigation/set_home'</br>
* serviceType: 'core_api/SetHome'



