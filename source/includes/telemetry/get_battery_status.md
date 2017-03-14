## Get Battery Status



> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.

ROS-Topic Name: /<namespace>/mavros/battery
ROS-Topic Type: sensor_msgs/BatteryState, below is its description

#Subscriber response : Battery Status 
Response structure:
uint8 POWER_SUPPLY_STATUS_UNKNOWN=0
uint8 POWER_SUPPLY_STATUS_CHARGING=1
uint8 POWER_SUPPLY_STATUS_DISCHARGING=2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING=3
uint8 POWER_SUPPLY_STATUS_FULL=4
uint8 POWER_SUPPLY_HEALTH_UNKNOWN=0
uint8 POWER_SUPPLY_HEALTH_GOOD=1
uint8 POWER_SUPPLY_HEALTH_OVERHEAT=2
uint8 POWER_SUPPLY_HEALTH_DEAD=3
uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE=4
uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE=5
uint8 POWER_SUPPLY_HEALTH_COLD=6
uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE=7
uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE=8
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN=0
uint8 POWER_SUPPLY_TECHNOLOGY_NIMH=1
uint8 POWER_SUPPLY_TECHNOLOGY_LION=2
uint8 POWER_SUPPLY_TECHNOLOGY_LIPO=3
uint8 POWER_SUPPLY_TECHNOLOGY_LIFE=4
uint8 POWER_SUPPLY_TECHNOLOGY_NICD=5
uint8 POWER_SUPPLY_TECHNOLOGY_LIMN=6
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 voltage
float32 current
float32 charge
float32 capacity
float32 design_capacity
float32 percentage
uint8 power_supply_status
uint8 power_supply_health
uint8 power_supply_technology
bool present
float32[] cell_voltage
string location
string serial_number
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

Type: Ros Topic
Name: /<namespace>/mavros/battery
Response Type: sensor_msgs/BatteryState
    uint8 POWER_SUPPLY_STATUS_UNKNOWN=0
    uint8 POWER_SUPPLY_STATUS_CHARGING=1
    uint8 POWER_SUPPLY_STATUS_DISCHARGING=2
    uint8 POWER_SUPPLY_STATUS_NOT_CHARGING=3
    uint8 POWER_SUPPLY_STATUS_FULL=4
    uint8 POWER_SUPPLY_HEALTH_UNKNOWN=0
    uint8 POWER_SUPPLY_HEALTH_GOOD=1
    uint8 POWER_SUPPLY_HEALTH_OVERHEAT=2
    uint8 POWER_SUPPLY_HEALTH_DEAD=3
    uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE=4
    uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE=5
    uint8 POWER_SUPPLY_HEALTH_COLD=6
    uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE=7
    uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE=8
    uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN=0
    uint8 POWER_SUPPLY_TECHNOLOGY_NIMH=1
    uint8 POWER_SUPPLY_TECHNOLOGY_LION=2
    uint8 POWER_SUPPLY_TECHNOLOGY_LIPO=3
    uint8 POWER_SUPPLY_TECHNOLOGY_LIFE=4
    uint8 POWER_SUPPLY_TECHNOLOGY_NICD=5
    uint8 POWER_SUPPLY_TECHNOLOGY_LIMN=6
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    float32 voltage
    float32 current
    float32 charge
    float32 capacity
    float32 design_capacity
    float32 percentage
    uint8 power_supply_status
    uint8 power_supply_health
    uint8 power_supply_technology
    bool present
    float32[] cell_voltage
    string location
    string serial_number

```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Topic
Name: /<namespace>/mavros/battery
Response Type: sensor_msgs/BatteryState
    uint8 POWER_SUPPLY_STATUS_UNKNOWN=0
    uint8 POWER_SUPPLY_STATUS_CHARGING=1
    uint8 POWER_SUPPLY_STATUS_DISCHARGING=2
    uint8 POWER_SUPPLY_STATUS_NOT_CHARGING=3
    uint8 POWER_SUPPLY_STATUS_FULL=4
    uint8 POWER_SUPPLY_HEALTH_UNKNOWN=0
    uint8 POWER_SUPPLY_HEALTH_GOOD=1
    uint8 POWER_SUPPLY_HEALTH_OVERHEAT=2
    uint8 POWER_SUPPLY_HEALTH_DEAD=3
    uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE=4
    uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE=5
    uint8 POWER_SUPPLY_HEALTH_COLD=6
    uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE=7
    uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE=8
    uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN=0
    uint8 POWER_SUPPLY_TECHNOLOGY_NIMH=1
    uint8 POWER_SUPPLY_TECHNOLOGY_LION=2
    uint8 POWER_SUPPLY_TECHNOLOGY_LIPO=3
    uint8 POWER_SUPPLY_TECHNOLOGY_LIFE=4
    uint8 POWER_SUPPLY_TECHNOLOGY_NICD=5
    uint8 POWER_SUPPLY_TECHNOLOGY_LIMN=6
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    float32 voltage
    float32 current
    float32 charge
    float32 capacity
    float32 design_capacity
    float32 percentage
    uint8 power_supply_status
    uint8 power_supply_health
    uint8 power_supply_technology
    bool present
    float32[] cell_voltage
    string location
    string serial_number

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/mavros/battery'

JSON Response:
{  voltage: Float,
    current: Float,
    remaining: Float}

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/mavros/battery',
messageType: 'sensor_msgs/BatteryState'

Response:
{   voltage: Float,
    current: Float,
    remaining: Float}

```


> Example

```shell
rostopic echo /flytpod/mavros/battery
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
from sensor_msgs.msg import BatteryState

# setup a subscriber and associate a callback function which will be called every time topic is updated.
topic_sub = rospy.Subscriber("/namespace/mavros/battery"), BatteryState, topic_callback)

# define the callback function which will print the values every time topic is updated
def topic_callback(data):
    voltage = data.voltage
    print voltage

# unsubscribe from a topic
topic_sub.unregister()  # unregister topic subscription
```

```javascript--REST
$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/mavros/battery",  
    success: function(data){
           console.log(data);
    }
};


```

```javascript--Websocket
var batteryData = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/mavros/battery',
    messageType : 'sensor_msgs/BatteryState'
});

var request = new ROSLIB.ServiceRequest({});

batteryData.subscribe(request, function(result) {
    console.log(result.data);
});
```


> Example response

```shell
header: 
  seq: 415
  stamp: 
    secs: 1489486630
    nsecs: 434288692
  frame_id: ''
voltage: 65.5350036621
current: 0.00999999977648
charge: nan
capacity: nan
design_capacity: nan
percentage: -0.00999999977648
power_supply_status: 2
power_supply_health: 0
power_supply_technology: 3
present: True
cell_voltage: []
location: id0
serial_number: ''
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
instance of sensor_msgs.msg.BatteryState class

```

```javascript--REST
{
    voltage: Float,
    current: Float,
    remaining: Float}


```

```javascript--Websocket
{
    voltage: Float,
    current: Float,
    remaining: Float}



```



###Description:

This API subscribes/poles battery status.  Please check API usage section below before using API.

###Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTFul, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Response:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    voltage | float | total voltage, Volts
    current | float | instantaneous current consumption, Amperes
    charge | float | Charge
    capacity | float | capacity
    percentage | float | percentage left
   
   
### ROS endpoint:
All the autopilot state / payload data in FlytOS is shared by ROS topics. Onboard topic subscribers in rospy / roscpp can subscribe to these topics. Take a look at roscpp and rospy API definition for response message structure. 

* Type: Ros Topic</br> 
* Name: /namespace/mavros/battery</br>
* Response Type: sensor_msgs/BatteryState

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice. All RESTful APIs can poll the data. For telemetry mode (continuous data stream) use websocket APIs.

* URL: ````GET http://<ip>/ros/<namespace>/mavros/battery````
* JSON Response:
{
    voltage: Float,
    current: Float,
    remaining: Float
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/mavros/battery'</br>
* messageType: 'sensor_msgs/BatteryState'

### API usage information:

* This API provides roll, pitch, yaw, rollspeed, pitchspeed, yawspeed information.
* Data returned is in NED frame.
