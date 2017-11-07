## Get Distance Sensor

> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Topic Name: /<namespace>/mavros/distance_sensor/lidarlite_pub
ROS-Topic Type: sensor_msgs/Range

Response structure:
    uint8 ULTRASOUND=0
    uint8 INFRARED=1
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    uint8 radiation_type
    float32 field_of_view
    float32 min_range
    float32 max_range
    float32 range


```

```cpp
Not Implemented
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from python.
Class: flyt_python.API.navigation
Function: get_distance_sensor()
Response: dist_sensor as described below.
 class dist_sensor:
    """
    Holds distance sensor data
    """
    radiation_type = 0
    field_of_view = 0.0
    min_range = 0.0
    max_range = 0.0
    range = 0.0

This API supports single poll mode only.
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

ROS-Topic Name: /<namespace>/mavros/distance_sensor/lidarlite_pub
ROS-Topic Type: sensor_msgs/Range

Response structure:
    uint8 ULTRASOUND=0
    uint8 INFRARED=1
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    uint8 radiation_type
    float32 field_of_view
    float32 min_range
    float32 max_range
    float32 range



```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

ROS-Topic Name: /<namespace>/mavros/distance_sensor/lidarlite_pub
ROS-Topic Type: sensor_msgs/Range

Response structure:
    uint8 ULTRASOUND=0
    uint8 INFRARED=1
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    uint8 radiation_type
    float32 field_of_view
    float32 min_range
    float32 max_range
    float32 range
```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/mavros/distance_sensor/lidarlite_pub'

JSON Response:
{   radiation_type: Int,
    field_of_view: Float,
    min_range: Float,  
    max_range: Float,
    range: Float
}

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/mavros/distance_sensor/lidarlite_pub',
messageType: 'sensor_msgs/Range'

Response:
{   radiation_type: Int,
    field_of_view: Float,
    min_range: Float,  
    max_range: Float,
    range: Float
}

```


> Example

```shell
rostopic echo /flytos/mavros/distance_sensor/lidarlite_pub
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
dist = drone.get_distance_sensor()

# Print the data
print dist.range, dist.max_range, dist.min_range, dist.field_of_view, dist.radiation_type
```

```cpp--ros
// Please refer to Roscpp documentation for sample subscriber nodes. http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
```

```python--ros
import rospy
from sensor_msgs.msgs import Range

# setup a subscriber and associate a callback function which will be called every time topic is updated.
topic_sub = rospy.Subscriber("/<namespace>/mavros/distance_sensor/lidarlite_pub"), Range, topic_callback)

# define the callback function which will print the values every time topic is updated
def topic_callback(data):
    dist = data.range
    print dist

# unsubscribe from a topic
topic_sub.unregister()  # unregister topic subscription
```

```javascript--REST
$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/mavros/distance_sensor/lidarlite_pub",  
    success: function(data){
           console.log(data);
    }
};


```

```javascript--Websocket
var distanceData = new ROSLIB.Topic({
    ros : ros,
    name : '/<namespace>/mavros/distance_sensor/lidarlite_pub',
    messageType : 'sensor_msgs/Range'
});


distanceData.subscribe(function(message) {
    console.log(message.data);
});
```


> Example response

```shell
instance of sensor_msgs/Range
```

```cpp
Not Implemented
```

```python
instance of class dist_sensor
```

```cpp--ros
```

```python--ros
instance of sensor_msgs.msgs.Range object

```

```javascript--REST
{
    radiation_type: Int,
    field_of_view: Float,
    min_range: Float,  
    max_range: Float,
    range: Float
}

```

```javascript--Websocket
{
    radiation_type: Int,
    field_of_view: Float,
    min_range: Float,  
    max_range: Float,
    range: Float
}


```

### Description:

This API subscribes/polls distance sensor data. Check API usage section below before using API.

### API usage information:

* This topic provides data from Lidarlite rangefinder, ultrasonic SONAR sensor, etc.
* This API will work on any px4 supported hardware.
* If you are using FlytPOD then check hardware and wiring sections in docs for wiring info.
* If you are using anything else than FlytPOD then refer to respective autopilot documentation for wiring info.

### Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Response:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    range | float | distance to ground in meters

### ROS endpoint:

All the autopilot state / payload data in FlytOS is shared by ROS topics. Onboard topic subscribers in rospy / roscpp can subscribe to these topics. Take a look at roscpp and rospy API definition for response message structure. 

* Type: `Ros Topic`
* Name: `/<namespace>/mavros/distance_sensor/lidarlite_pub`
* Response Type: `sensor_msgs/Range`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port **80**. RESTful APIs can be called from remote platform of your choice. All RESTful APIs can poll the data. For telemetry mode (continuous data stream) use websocket APIs.

* URL: `GET http://<ip>/ros/<namespace>/mavros/distance_sensor/lidarlite_pub`
* JSON Response:
`{
    radiation_type: Int,
    field_of_view: Float,
    min_range: Float,  
    max_range: Float,
    range: Float
}`

### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 

Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace>/mavros/distance_sensor/lidarlite_pub`
* messageType: `sensor_msgs/Range`
