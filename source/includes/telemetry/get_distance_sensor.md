## Get Distance Sensor



> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

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

NotImplemented
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
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
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
rostopic echo /flytpod/mavros/distance_sensor/lidarlite_pub
```

```cpp
Not Implemented
```

```python
NotImplemented
```

```cpp--ros
// Please refer to Roscpp documenation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c%2B%2B)
```

```python--ros
from sensor_msgs.msgs import Range

# setup a subscriber and associate a callback function which will be called every time topic is updated.
topic_sub = rospy.Subscriber("/namespace/mavros/distance_sensor/lidarlite_pub"), Range, topic_callback)

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
var distanceData = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/mavros/distance_sensor/lidarlite_pub',
    messageType : 'sensor_msgs/Range'
});

var request = new ROSLIB.ServiceRequest({});

distanceData.subscribe(request, function(result) {
    console.log(result.data);
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
NotImplemented
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



###Description:

This API subscribes/polls distance sensor data.  Please check API usage section below before using API.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTFul, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Response:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    range | float | distance to ground in meters
    
    
### ROS endpoint:
All the autopilot state / payload data in FlytOS is shared by ROS topics. Onboard topic subscribers in rospy / roscpp can subscribe to these topics. Take a look at roscpp and rospy API definition for response message structure. 

* Type: Ros Topic</br> 
* Name: /namespace/mavros/distance_sensor/lidarlite_pub</br>
* Response Type: sensor_msgs/Range

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice. All RESTful APIs can poll the data. For telemetry mode (continuous data stream) use websocket APIs.

* URL: ````GET http://<ip>/ros/<namespace>/mavros/distance_sensor/lidarlite_pub````
* JSON Response:
{
    radiation_type: Int,
    field_of_view: Float,
    min_range: Float,  
    max_range: Float,
    range: Float
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/mavros/distance_sensor/lidarlite_pub'</br>
* messageType: 'sensor_msgs/Range'

### API usage information:

* This topic provides data from Lidarlite rangefinder, ultrasonic SONAR sensor, etc.
* This API will work on any px4 supported hardware.
* If you are using FlytPOD then check hardware and wiring sections in docs for wiring info.
* If you are using anything else than FlytPOD then refer to respective autopilot documentation for wiring info.