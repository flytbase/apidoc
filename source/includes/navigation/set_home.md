## Set Home


> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

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
```

```cpp
No CPP API available.
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

NotImplemented
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/set_home()
call srv:
    :float64 lat
    :float64 lon
    :float64 alt
    :bool set_current
response srv: bool success
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/navigation/set_home()
call srv:
    :float64 lat
    :float64 lon
    :float64 alt
    :bool set_current
response srv: bool success

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
{   success: Boolean, }

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
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
{   success: Boolean, }


```


> Example

```shell
rosservice call /flytsim/navigation/set_home "{lat: 73.25564541, lon: 18.2165632, alt: 2.0, set_current: false}"  
```

```cpp
```

```python
NotImplemented
```

```cpp--ros
#include <core_api/SetHome.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::SetHome>("navigation/set_home");
core_api::SetHome srv;

srv.request.lat = 73.25564541;
srv.request.lon = 18.2165632;
srv.request.alt = 2.00;
srv.request.set_current = false;
client.call(srv);
success = srv.response.success;
```

```python--ros
def setpoint_local_position(lx, ly, lz, yaw, tolerance= 0.0, async = False, relative= False, yaw_rate_valid= False, body_frame= False):
    rospy.wait_for_service('namespace/navigation/set_home')
    try:
        handle = rospy.ServiceProxy('namespace/navigation/set_home', SetHome)
        twist = {'header': {'seq': seq, 'stamp': {'secs': sec, 'nsecs': nsec}, 'frame_id': f_id}, 'twist': {'linear': {'x': lx, 'y': ly, 'z': lz}, 'angular': {'z': yaw}}}
        resp = handle(twist, tolerance, async, relative, yaw_rate_valid, body_frame)
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("pos set service call failed %s", e)

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
NotImplemented
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
    x, y, z | float | Position Setpoint in NED-Frame (in body-frame if body_frame=true)
    yaw | float | Yaw Setpoint in radians
    yaw_valid | bool | Must be set to true, if yaw 
    tolerance | float | Acceptance radius in meters, default value=1.0m 
    relative | bool | If true, position setpoints relative to current position is sent
    async | bool | If true, asynchronous mode is set
    body_frame | bool | If true, position setpoints are relative with respect to body frame
    
    Output:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful

### ROS endpoint:
Navigation APIs in FlytOS are derived from / wrapped around the core navigation services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy api definition for message structure. 

* Type: Ros Service</br> 
* Name: /namespace/navigation/set_home</br>
* Service Type: SetHome

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ````POST http://<ip>/ros/<namespace>/navigation/set_home````
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
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/navigation/set_home'</br>
* serviceType: 'core_api/SetHome'


### API usage information:
Note: You can either set body_frame or relative flag. If both are set, body_frame takes precedence.

Tip: Asynchronous mode - The API call would return as soon as the command has been sent to the autopilot, irrespective of whether the vehicle has reached the given setpoint or not.

Tip: Synchronous mode - The API call would wait for the function to return, which happens when either the position setpoint is reached or timeout=30secs is over.

