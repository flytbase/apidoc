## Obstacle Detection

> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

ROS-Topic Name: /<namespace>/dji_msdk/obstacle_distance
ROS-Topic Type: core_api/ObstacleData, below is its description

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    float32[] front
    float32[] back
    uint8[] front_warning_level
    uint8[] back_warning_level
    bool on_hold
    bool is_enabled
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

// Not Implemented
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

# Not Implemented
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

ROS-Topic Name: /<namespace>/dji_msdk/obstacle_distance
ROS-Topic Type: core_api/ObstacleData, below is its description

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    float32[] front
    float32[] back
    uint8[] front_warning_level
    uint8[] back_warning_level
    bool on_hold
    bool is_enabled
```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

ROS-Topic Name: /<namespace>/dji_msdk/obstacle_distance
ROS-Topic Type: core_api/ObstacleData, below is its description

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    float32[] front
    float32[] back
    uint8[] front_warning_level
    uint8[] back_warning_level
    bool on_hold
    bool is_enabled
```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/dji_msdk/obstacle_distance'

JSON Response:
{
    front: FloatArray,
    back: FloatArray,
    front_warning_level: IntArray,
    back_warning_level: IntArray,
    on_hold: Bool,
    is_enabled: Bool
}
```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/dji_msdk/obstacle_distance',
serviceType: 'core_api/ObstacleData'

Response:
{
    front: FloatArray,
    back: FloatArray,
    front_warning_level: IntArray,
    back_warning_level: IntArray,
    on_hold: Bool,
    is_enabled: Bool
}
```

> Example

```shell
Not Implemented
```

```cpp
Not Implemented
```

```python
Not Implemented
```

```cpp--ros
Not Implemented
```

```python--ros
Not Implemented
```

```javascript--REST
$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/dji_msdk/obstacle_distance",
    success: function(data) {
           console.log(data);
    }
});
```

```javascript--Websocket
var obsData = new ROSLIB.Topic({
    ros : ros,
    name : '/<namespace>/dji_msdk/obstacle_distance',
    messageType : 'core_api/ObstacleData',
    throttle_rate: 200
});

obsData.subscribe(function(message) {
    console.log(message);
});
```


> Example response

```shell
Not Implemented
```

```cpp
Not Implemented
```

```python
Not Implemented
```

```cpp--ros
Not Implemented
```

```python--ros
Not Implemented
```

```javascript--REST
Not Implemented
```

```javascript--Websocket
front: [16.43000030517578, 14.010000228881836, 0.8700000047683716, 0.8700000047683716]
back: [0.8700000047683716, 0.8700000047683716, 0.8700000047683716, 15.569999694824219]
front_warning_level: [0, 0, 5, 5],
back_warning_level: [5, 5, 5, 0],
on_hold: False
is_enabled: False
```

### Description:

This API publishes detected obstacle data of a supported DJI drone.

<aside class="warning">
    This API will **ONLY** work with FlytOS mobile app.
</aside>

### Parameters:
    
    Following parameters are applicable for onboard C++ and Python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    front | floatArray | array of detected obstacle distance (in m) for every quadrant of front sensor from left to right
    back | floatArray | array of detected obstacle distance (in m) for every quadrant of back sensor from left to right
    front_warning_level | intArray | array of warning levels for every quadrant of front sensor from left to right. Values in range (0 - 5) or 15. 0 being farthest and 5 being closest. If the sensor data is invalid, this value will be 15.
    back_warning_level | intArray | array of detected obstacle distance (in m) for every quadrant of back sensor from left to right
    on_hold | bool | stores state if autopilot has engaged obstacle avoidance
    is_enabled | bool | stores state if collision avoidance is enabled or not
    

### ROS endpoint:

Payload APIs in FlytOS are derived from / wrapped around the core services available in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: `Ros Service`
* Name: `/<namespace>/dji_msdk/obstacle_distance`
* Service Type: `core_api/ObstacleData`

### Websocket endpoint:

Websocket APIs can be called from javascript using [roslibjs library](https://github.com/RobotWebTools/roslibjs).

Java websocket clients are supported using [rosjava](http://wiki.ros.org/rosjava).

* name: `/<namespace>/dji_msdk/obstacle_distance`
* serviceType: `core_api/ObstacleData`
