## Get VFR HUD

> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Topic Name: /<namespace>/mavros/vfr_hud
ROS-Topic Type: mavros_msgs/VFR_HUD

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    float32 airspeed
    float32 groundspeed
    int16 heading
    float32 throttle
    float32 altitude
    float32 climb

```

```cpp
Not Implemented
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from python.
Class: flyt_python.API.navigation
Function: get_vfr_hud()

Response: vfr_hud as described below.
    class vfr_hud:
    	'''
   	 Holds data for VFR HUD
   	 '''
    	airspeed = 0.0
   	groundspeed = 0.0
    	heading = 0.0
    	throttle = 0.0
    	altitude = 0.0
    	climb = 0.0

This API supports single poll mode only.
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Topic
Name: /<namespace>/mavros/mavros_msgs/vfr_hud

Response structure: mavros_msgs/VFR_HUD
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    float32 airspeed
    float32 groundspeed
    int16 heading
    float32 throttle
    float32 altitude
    float32 climb



```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Topic
Name: /<namespace>/mavros/mavros_msgs/vfr_hud

Response structure: mavros_msgs/VFR_HUD
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    float32 airspeed
    float32 groundspeed
    int16 heading
    float32 throttle
    float32 altitude
    float32 climb
```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/mavros/vfr_hud'

JSON Response:
{   airspeed:Float,
    groundspeed: Float,
    heading: Integer,
    throttle: Float,
    altitude: Float,
    climb: Float
}

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/mavros/vfr_hud',
messageType: 'mavros_msgs/VFR_HUD'

Response:
{   airspeed:Float,
    groundspeed: Float,
    heading: Integer,
    throttle: Float,
    altitude: Float,
    climb: Float
}

```

```python--flyt_python

# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.flyt_python.DroneApiConnector

Function: get_vfr_hud()

```


> Example

```shell
rostopic echo /flytos/mavros/mavros_msgs/vfr_hud
```

```cpp
Not Implemented
```

```python
# create flyt_python navigation class instance
from flyt_python import API
drone = API.navigation()
# wait for interface to initialize
time.sleep(3.0)

# Poll data
vfr = drone.get_vfr_hud()
# Print the data
print vfr.throttle, vfr.groundspeed, vfr.airspeed, vfr.altitude, vfr.climb, vfr.heading
```

```cpp--ros
// Please refer to Roscpp documentation for sample subscriber nodes. http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
```

```python--ros
import rospy
from mavros_msgs.msgs import VFR_HUD

# setup a subscriber and associate a callback function which will be called every time topic is updated.
topic_sub = rospy.Subscriber("/<namespace>/mavros/vfr_hud"), VFR_HUD, topic_callback)

# define the callback function which will print the values every time topic is updated
def topic_callback(data):
    airspeed = data.airspeed
    print airspeed

# unsubscribe from a topic
topic_sub.unregister()  # unregister topic subscription
```

```javascript--REST
$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/mavros/vfr_hud",  
    success: function(data){
           console.log(data);
    }
});


```

```javascript--Websocket
var vfrHUDData = new ROSLIB.Topic({
    ros : ros,
    name : '/<namespace>/mavros/vfr_hud',
    messageType : 'mavros_msgs/VFR_HUD'
});

vfrHUDData.subscribe(function(message) {
    console.log(message.data);
});
```

```python--flyt_python 
from flyt_python.flyt_python import DroneApiConnector
token = ''                      # Personal Access Token
vehicle_id = ''                 # Vehicle ID

#create an instance of class DroneApiConnector
drone = DroneApiConnector(token,vehicle_id,ip_address='localhost' wait_for_drone_response =True)
drone.connect()
    
drone.get_vfr_hud()

drone.disconnect()
```
> Example response

```shell
header: 
  seq: 3664
  stamp: 
    secs: 1489486597
    nsecs: 915001340
  frame_id: ''
airspeed: 0.0
groundspeed: 0.0
heading: 289
throttle: 0.0
altitude: 601.289001465
climb: -0.0
```

```cpp
Not Implemented
```

```python
instance of class vfr_hud
```

```cpp--ros
```

```python--ros
instance of mavros_msgs.msgs.VFR_HUD class

```

```javascript--REST
{
    airspeed:Float,
    groundspeed: Float,
    heading: Integer,
    throttle: Float,
    altitude: Float,
    climb: Float
}

```

```javascript--Websocket
{
    airspeed:Float,
    groundspeed: Float,
    heading: Integer,
    throttle: Float,
    altitude: Float,
    climb: Float
}
```
```python--flyt_python
{
    airspeed:Float,
    groundspeed: Float,
    heading: Integer,
    throttle: Float,
    altitude: Float,
    climb: Float
}
```


### Description:

This API subscribes/polls VFR HUD data. Please check API usage section below before using API.

### Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Response:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    airspeed | float | airspeed in m/s
    groundspeed | float | groundspeed in m/s
    heading | int16 | yaw angle in degrees (NED frame)
    throttle | float | throttle
    altitude | float | altitude
    climb | float | climb

### API usage information:

* airspeed data is the data from airspeed sensor.

### ROS endpoint:

All the autopilot state/payload data in FlytOS is shared by ROS topics. Onboard topic subscribers in rospy/roscpp can subscribe to these topics. Take a look at roscpp and rospy API definition for response message structure. 

* Type: `Ros Topic`
* Name: `/<namespace>/mavros/vfr_hud`
* Response Type: `mavros_msgs/VFR_HUD`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port **80**. RESTful APIs can be called from remote platform of your choice. All RESTful APIs can poll the data. For telemetry mode (continuous data stream) use websocket APIs.

* URL: `GET http://<ip>/ros/<namespace>/mavros/vfr_hud`
* JSON Response:
`{
    airspeed:Float,
    groundspeed: Float,
    heading: Integer,
    throttle: Float,
    altitude: Float,
    climb: Float
}`

### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 

Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace>/mavros/vfr_hud`
* messageType: `mavros_msgs/VFR_HUD`
