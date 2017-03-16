# Payload APIs

## Get ADC payload



> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Topic Name: /<namespace>/mavros/payload_adc
ROS-Topic Type: mavros_msgs/PayloadADC

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    float32[2] adc_voltage
    uint8 adc_updated

```

```cpp
// CPP API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from cpp.

Not Implemented
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from python.

NotImplemented
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

ROS-Topic Name: /<namespace>/mavros/payload_adc
ROS-Topic Type: mavros_msgs/PayloadADC

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    float32[2] adc_voltage
    uint8 adc_updated



```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

ROS-Topic Name: /<namespace>/mavros/payload_adc
ROS-Topic Type: mavros_msgs/PayloadADC

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    float32[2] adc_voltage
    uint8 adc_updated
```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/mavros/payload_adc'

JSON Response:
{   adc_voltage: Float[2],
    adc_updated: Int
}

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/mavros/payload_adc',
messageType: 'mavros_msgs/PayloadADC'

Response:
{   adc_voltage: Float[2],
    adc_updated: Int
}

```


> Example

```shell
rostopic echo /flytpod/mavros/payload_adc

#sends (x,y,z)=(1.0,3.5,-5.0)(m), yaw=0.12rad, relative=false, async=false, yaw_valid=true, body_frame=false
#default value of tolerance=1.0m if left at 0    
```

```cpp
Not Implemented
```

```python
NotImplemented
```

```cpp--ros
#include <core_api/PositionSet.h>

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<core_api::PositionSet>("navigation/position_set");
core_api::PositionSet srv;

srv.request.twist.twist.angular.z = 0.12;
srv.request.twist.twist.linear.x = 1.0;
srv.request.twist.twist.linear.y = 3.5;
srv.request.twist.twist.linear.z = -5.0;
srv.request.tolerance = 5.0;
srv.request.async = false;
srv.request.yaw_valid = true;
srv.request.relative = false;
srv.request.body_frame = false;
client.call(srv);
success = srv.response.success;

//sends (x,y,z)=(1.0,3.5,-5.0)(m), yaw=0.12rad, tolerance=5.0m, relative=false, async=false, yaw_valid=true, body_frame=false
```

```python--ros
from mavros_msgs.msgs import PayloadADC
# setup a subscriber and associate a callback function which will be called every time topic is updated.
topic_sub = rospy.Subscriber("/namespace/mavros/payload_adc"), PayloadADC, topic_callback)

# define the callback function which will print the values every time topic is updated
def topic_callback(data):
    adc1,adc2 = data.adc_voltage[0],data.adc_voltage[1]
    print adc1, adc2

# unsubscribe from a topic
topic_sub.unregister()  # unregister topic subscription
```

```javascript--REST
$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/mavros/payload_adc",  
    success: function(data){
           console.log(data);
    }
};


```

```javascript--Websocket
var adcData = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/mavros/payload_adc',
    messageType : 'mavros_msgs/PayloadADC'
});

var request = new ROSLIB.ServiceRequest({});

adcData.subscribe(request, function(result) {
    console.log(result.data);
});
```


> Example response

```shell
    adc_voltage: Float[2],
    adc_updated: Int
```

```cpp
Not Implemented
```

```python
NotImplemented
```

```cpp--ros
success: True
```

```python--ros
instance of mavros_msgs.msgs.PayloadADC object

```

```javascript--REST
{
    adc_voltage: Float[2],
    adc_updated: Int
}

```

```javascript--Websocket
{
    adc_voltage: Float[2],
    adc_updated: Int
}


```



###Description:

This API subscribes/polls the ADC payload data. This API is limited to FlytPOD only. Please see usage information section below before using the API.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTFul, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Response:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    adc_voltage | array (2*1) of floats | array of adc voltage readings in volts.
    adc_updated | uint8 | Bitmask indicating updated channel. possible values: [0,1,2,3]
    
    
### ROS endpoint:
All the autopilot state / payload data in FlytOS is shared by ROS topics. Onboard topic subscribers in rospy / roscpp can subscribe to these topics. Take a look at roscpp and rospy API definition for response message structure. 

* Type: Ros Topic</br> 
* Name: /namespace/mavros/payload_adc</br>
* Response Type: mavros_msgs/PayloadADC

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice. All RESTful APIs can poll the data. For telemetry mode (continuous data stream) use websocket APIs.

* URL: ````GET http://<ip>/ros/<namespace>/mavros/payload_adc````
* JSON Response:
{
    adc_voltage: Float[2],
    adc_updated: Int
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/mavros/payload_adc'</br>
* messageType: 'mavros_msgs/PayloadADC'

### API usage information:

* This API works only on FlytPOD.  
* There are two ADC channels available on back IO panel of FlytPOD. Refer to hardware connections section in FlytPOD documentation for more info on wiring.
* ADC channel operational input voltage range is 0 to 3.3 V.