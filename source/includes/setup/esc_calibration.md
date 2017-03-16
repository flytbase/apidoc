## ESC Calibration


> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/setup/esc_calibration
ROS-Service Type: core_api/EscCalibration, below is its description
ReqStructure: 
    int8 CALIBRATION_STATE_SET_PWM_MAX = 1
    int8 CALIBRATION_STATE_SET_PWM_MIN = 2
    int8 CALIBRATION_STATE_CANCEL = 3
    float32 pwm_min
    float32 pwm_max
    int8 num_of_actuators
    int8 calibration_state
    ---
    bool success

```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

Not Implemented
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

NotImplemented
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

ROS-Service Name: /<namespace>/setup/esc_calibration
ROS-Service Type: core_api/EscCalibration, below is its description
ReqStructure: 
    int8 CALIBRATION_STATE_SET_PWM_MAX = 1
    int8 CALIBRATION_STATE_SET_PWM_MIN = 2
    int8 CALIBRATION_STATE_CANCEL = 3
    float32 pwm_min
    float32 pwm_max
    int8 num_of_actuators
    int8 calibration_state
    ---
    bool success

```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

ROS-Service Name: /<namespace>/setup/esc_calibration
ROS-Service Type: core_api/EscCalibration, below is its description
ReqStructure: 
    int8 CALIBRATION_STATE_SET_PWM_MAX = 1
    int8 CALIBRATION_STATE_SET_PWM_MIN = 2
    int8 CALIBRATION_STATE_CANCEL = 3
    float32 pwm_min
    float32 pwm_max
    int8 num_of_actuators
    int8 calibration_state
    ---
    bool success

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/setup/esc_calibration'

JSON Request:
{   pwm_min: Float,
    pwm_max: Float,
    num_of_actuators: Int,
    calibration_state: Int }

JSON Response:
{   success: Boolean, }

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/setup/esc_calibration',
serviceType: 'core_api/EscCalibration'

Request:
{   pwm_min: Float,
    pwm_max: Float,
    num_of_actuators: Int,
    calibration_state: Int }

Response:
{   success: Boolean, }


```


> Example

```shell

# Refer to rosservice command line api documentation for sample service calls. http://wiki.ros.org/rosservice
    
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
# Please refer to Rospy documenation for sample service clients. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
 ```

```javascript--REST
var  msgdata={};
msgdata["pwm_min"]=1000.00;
msgdata["pwm_max"]=2000.00;
msgdata["num_of_actuators"]=4;
msgdata["calibration_state"]=2;

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/setup/esc_calibration",  
    success: function(data){
           console.log(data.success);
    }
};

```

```javascript--Websocket
var escCalibration = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/setup/esc_calibration',
    serviceType : 'core_api/EscCalibration'
});

var request = new ROSLIB.ServiceRequest({
    pwm_min: 1000.00,
    pwm_max: 2000.00,
    num_of_actuators: 4,
    calibration_state: 2
});

escCalibration.callService(request, function(result) {
    console.log('Result for service call on '
      + escCalibration.name
      + ': '
      + result.success);
});
```


> Example response

```shell
success: true
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
This API helps calibrate ESCs.

###Parameters:
    
    Following parameters are applicable in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    pwm_min | float | Min PWM value to be expected
    pwm_max | float | Max PWM value to be expected
    num_of_actuators | Int | Number of actuator in the frame.
    calibration_state | Int | 1/2/3.
    
    Output:
    
    Parameter | type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful

### ROS endpoint:
APIs in FlytOS are derived from / wrapped around the core services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy api definition for message structure. 

* Type: Ros Service</br> 
* Name: /namespace/setup/esc_calibration</br>
* Service Type: EscCalibration

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ````POST http://<ip>/ros/<namespace>/setup/esc_calibration````
* JSON Request:
{
    pwm_min: Float,
    pwm_max: Float,
    num_of_actuators: Int,
    calibration_state: Int
}
* JSON Response:
{
    success: Boolean
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/setup/esc_calibration'</br>
* serviceType: 'core_api/EscCalibration'


<!-- ### API usage information:
Note: You can either set body_frame or relative flag. If both are set, body_frame takes precedence.

Tip: Asynchronous mode - The API call would return as soon as the command has been sent to the autopilot, irrespective of whether the vehicle has reached the given setpoint or not.

Tip: Synchronous mode - The API call would wait for the function to return, which happens when either the position setpoint is reached or timeout=30secs is over.
 -->
