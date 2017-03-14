# Module Calibration


> Definition

```shell
# API call described below requires shell access, either login to the device using desktop or use ssh for remote login.


Type: Ros Service
Name: /<namespace>/setup/module_calibration()
MsgType: core_api/ModuleCalibration
MsgStructure:
    uint8 STOP = 0
    uint8 ACCELEROMETER = 1
    uint8 GYROSCOPE = 2
    uint8 MAGNETOMETER = 3
    uint8 RC = 4
    uint8 RC_TRIM = 5
    uint8 RC_STOP = 6
    uint8 LEVEL = 7
    uint8 AIRSPEED = 8
    
    int8 module_calibrate
    ---
    bool success
```

```cpp
// C++ API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from C++.

NotImplemented
```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

NotImplemented
```

```cpp--ros
// ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/setup/module_calibration()
MsgType: core_api/ModuleCalibration
MsgStructure:
    uint8 STOP = 0
    uint8 ACCELEROMETER = 1
    uint8 GYROSCOPE = 2
    uint8 MAGNETOMETER = 3
    uint8 RC = 4
    uint8 RC_TRIM = 5
    uint8 RC_STOP = 6
    uint8 LEVEL = 7
    uint8 AIRSPEED = 8
    
    int8 module_calibrate
    ---
    bool success

```

```python--ros
# ROS services and topics are accessible from onboard scripts only.

Type: Ros Service
Name: /<namespace>/setup/module_calibration()
MsgType: core_api/ModuleCalibration
MsgStructure:
    uint8 STOP = 0
    uint8 ACCELEROMETER = 1
    uint8 GYROSCOPE = 2
    uint8 MAGNETOMETER = 3
    uint8 RC = 4
    uint8 RC_TRIM = 5
    uint8 RC_STOP = 6
    uint8 LEVEL = 7
    uint8 AIRSPEED = 8
    
    int8 module_calibrate
    ---
    bool success

```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/setup/module_calibration'

JSON Request:
{   module_calibrate: Int }

JSON Response:
{   success: Boolean, }

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialisng 
API and and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/setup/module_calibration',
serviceType: 'core_api/ModuleCalibration'

Request:
{   module_calibrate: Int }

Response:
{   success: Boolean, }


```


> Example

```shell

// Refer to rosservice command line api documentation for sample service calls. http://wiki.ros.org/rosservice
    
```

```cpp
NotImplemented
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
msgdata["module_calibrate"]=2;

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/setup/module_calibration",  
    success: function(data){
           console.log(data.success);
    }
};

```

```javascript--Websocket
var moduleCalibration = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/setup/module_calibration',
    serviceType : 'core_api/ModuleCalibration'
});

var request = new ROSLIB.ServiceRequest({
    module_calibrate: Int
});

moduleCalibration.callService(request, function(result) {
    console.log('Result for service call on '
      + moduleCalibration.name
      + ': '
      + result.success);
});
```


> Example response

```shell

```

```cpp
NotImplemented
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
This API sends local position setpoint command to the autopilot. Additionally, you can send yaw setpoint (yaw_valid flag must be set true) to the vehicle as well. Some abstract features have been added, such as tolerance/acceptance-radius, synchronous/asynchronous mode, sending setpoints relative to current position (relative flag must be set true), sending setpoints relative to current body frame (body_frame flag must be set true).
This command commands the vehicle to go to a specified location and hover. It overrides any previous mission being carried out and starts hovering.

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
* Name: /namespace/setup/module_calibration</br>
* Service Type: ModuleCalibration

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ````POST http://<ip>/ros/<namespace>/setup/module_calibration````
* JSON Request:
{
    module_calibrate: Int
}
* JSON Response:
{
    success: Boolean
}


### Websocket endpoint:
Websocket APIs can be called from javascript using  [roslibjs library.](https://github.com/RobotWebTools/roslibjs) 
Java websocket clients are supported using [rosjava.](http://wiki.ros.org/rosjava)

* name: '/namespace/setup/module_calibration'</br>
* serviceType: 'core_api/ModuleCalibration'


### API usage information:
Note: You can either set body_frame or relative flag. If both are set, body_frame takes precedence.

Tip: Asynchronous mode - The API call would return as soon as the command has been sent to the autopilot, irrespective of whether the vehicle has reached the given setpoint or not.

Tip: Synchronous mode - The API call would wait for the function to return, which happens when either the position setpoint is reached or timeout=30secs is over.

