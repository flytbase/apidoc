## Get Vehicle State

> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Topic Name: /<namespace>/flyt/state
ROS-Topic Type: mavros_msgs/State

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    bool connected
    bool armed
    bool guided
    string mode
    uint8 mav_type
    uint8 mav_autopilot
    uint8 mav_sys_status

```

```cpp
// CPP API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from cpp.

Function Definition: sysSubscribe(Navigation::vehicle_state,vehicleModeCb);

Arguments:
    vehicle_state: This argument selects vehicle state topic to be subscribed
    vehicleModeCb: Callback function for the subscribed vehicle state messages

Returns: Vehicle state in ros mavros_msgs::State message structure
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    bool connected
    bool armed
    bool guided
    string mode
    uint8 mav_type
    uint8 mav_autopilot
    uint8 mav_sys_status

```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from python.

# Python API for vehicle state is split into three APIs

# Get full vehicle state data
Class: flyt_python.API.navigation
Function: get_vehicle_state()
Response: vehicle_state as described below.
class vehicle_state:
    """
    Holds vehicle state data
    """
    connected = False
    armed = False
    guided = False
    mode = 'String'
    mav_type = 0
    mav_autopilot = 0
    mav_sys_status = 0
 
This API supports single poll mode only.

# Check arm status
Class: flyt_python.api.navigation
Function Definition: is_armed()
Arguments: None
return: Boolean

# Check vehicle mode
Class: flyt_python.api.navigation
Function Definition: get_vehicle_mode()
Arguments: None
return: string

```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

ROS-Topic Name: /<namespace>/flyt/state
ROS-Topic Type: mavros_msgs/State

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    bool connected
    bool armed
    bool guided
    string mode
    uint8 mav_type
    uint8 mav_autopilot
    uint8 mav_sys_status

```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

ROS-Topic Name: /<namespace>/flyt/state
ROS-Topic Type: mavros_msgs/State

Response structure:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    bool connected
    bool armed
    bool guided
    string mode
    uint8 mav_type
    uint8 mav_autopilot
    uint8 mav_sys_status
```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/flyt/state'

JSON Response:
{   connected: Boolean,
    armed: Boolean,
    guided: Boolean,
    mode: String,
    mav_type: Int,
    mav_autopilot: Int,
    mav_sys_status: Int
}

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/flyt/state',
messageType: 'mavros_msgs/State'

Response:
{   connected: Boolean,
    armed: Boolean,
    guided: Boolean,
    mode: String,
    mav_type: Int,
    mav_autopilot: Int,
    mav_sys_status: Int
}

```
```python--flyt_python

# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Class: flyt_python.flyt_python.DroneApiConnector

Function: get_vehicle_state()

```

> Example

```shell
rostopic echo /flytos/flyt/state
```

```cpp
#include <cpp_api/navigation_bridge.h>

Navigation nav;
mavros_msgs::State vehicle_state;

void attitudeQuatCb(void *_vehicle_state)
{
    vehicle_state = * (mavros_msgs::State*)(_vehicle_state);
}
nav.sysSubscribe(Navigation::vehicle_state,vehicleModeCb);

std::cout << vehicle_state << std::endl;
```

```python
# create flyt_python navigation class instance

from flyt_python import api
drone = api.navigation()
time.sleep(3.0)
# Poll data
st = drone.get_vehicle_state()

# Print the data
print st.connected, st.armed, st.guided, st.mode, st.mav_type, st.mav_autopilot, st.mav_sys_status

# get arm status
print drone.is_armed()
print drone.get_vehicle_mode()

```

```cpp--ros
// Please refer to Roscpp documentation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c%2B%2B)
```

```python--ros
import rospy
from mavros_msgs.msgs import State
# setup a subscriber and associate a callback function which will be called every time topic is updated.
topic_sub = rospy.Subscriber("/<namespace>/flyt/state"), State, topic_callback)

# define the callback function which will print the values every time topic is updated
def topic_callback(data):
    mode, is_armed = data.mode, data.armed
    print mode, is_armed

# unsubscribe from a topic
topic_sub.unregister()  # unregister topic subscription
```

```javascript--REST
$.ajax({
    type: "GET",
    dataType: "json",
    url: "http://<ip>/ros/<namespace>/flyt/state",  
    success: function(data){
           console.log(data);
    }
});


```

```javascript--Websocket
var stateData = new ROSLIB.Topic({
    ros : ros,
    name : '/<namespace>/flyt/state',
    messageType : 'mavros_msgs/State'
});

stateData.subscribe(function(message) {
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
    
drone.get_vehicle_state()

drone.disconnect()
```
> Example response

```shell
header: 
  seq: 1666
  stamp: 
    secs: 1489487905
    nsecs: 864919940
  frame_id: ''
connected: True
armed: False
guided: False
mode: RC|MANUAL
mav_type: 2
mav_autopilot: 12
mav_sys_status: 0
```

```cpp
Instance of mavros_msgs::State class
```

```python
instance of class vehicle_state
True
MANUAL
```

```cpp--ros
```

```python--ros
True MANUAL
```

```javascript--REST
{
    connected: Boolean,
    armed: Boolean,
    guided: Boolean,
    mode: String,
    mav_type: Int,
    mav_autopilot: Int,
    mav_sys_status: Int
}

```

```javascript--Websocket
{
    connected: Boolean,
    armed: Boolean,
    guided: Boolean,
    mode: String,
    mav_type: Int,
    mav_autopilot: Int,
    mav_sys_status: Int
}
```

```python--flyt_python
{
    connected: Boolean,
    armed: Boolean,
    guided: Boolean,
    mode: String,
    mav_type: Int,
    mav_autopilot: Int,
    mav_sys_status: Int
}
```

### Description:

This API subscribes/polls the vehicle state data. Check usage information section below before using the API.

### Parameters:
    
    Following parameters are applicable for onboard cpp and python scripts. Scroll down for their counterparts in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Response:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    mode | string | autopilot flight mode e.g. MANUAL, APICTL
    armed | boolean | Vehicle arm status. Armed if True and disarmed if False.

### API usage information:

* This API provides mode and arm status.
* All navigation API's work only in Offboard / APICTL mode. So checking the mode before firing mission critical commands is advised.
* This API only allows to read the mode and arm status. 

### ROS endpoint:

All the autopilot state/payload data in FlytOS is shared by ROS topics. Onboard topic subscribers in rospy / roscpp can subscribe to these topics. Take a look at roscpp and rospy API definition for response message structure. 

* Type: `Ros Topic`
* Name: `/<namespace>/flyt/state`
* Response Type: `mavros_msgs/State`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port **80**. RESTful APIs can be called from remote platform of your choice. All RESTful APIs can poll the data. For telemetry mode (continuous data stream) use websocket APIs.

* URL: `GET http://<ip>/ros/<namespace>/flyt/state`
* JSON Response:
`{
    connected: Boolean,
    armed: Boolean,
    guided: Boolean,
    mode: String,
    mav_type: Int,
    mav_autopilot: Int,
    mav_sys_status: Int
}`

### Websocket endpoint:

Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 

Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: `/<namespace>/flyt/state`
* messageType: `mavros_msgs/State`
