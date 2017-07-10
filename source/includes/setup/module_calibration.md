## Module Calibration


> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.


Type: Ros Service
Name: /<namespace>/setup/module_calibration
MsgType: core_api/ModuleCalibration

#request
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

#response
bool success
string message
```

```cpp
//Not Implemented
```

```python
#Not Implemented
```

```cpp--ros
//Not Recommended
```

```python--ros
#Not Recommended
```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/ros/<namespace>/setup/module_calibration'

JSON Request:
{   module_calibrate: Int }

JSON Response:
{   success: Boolean,
    message: String, }

```

```javascript--Websocket
This is a Websocket call for the API. Make sure you 
initialise the websocket using websocket initialising 
API and replace namespace with the namespace of 
the FlytOS running device before calling the API 
with websocket.

name: '/<namespace>/setup/module_calibration',
serviceType: 'core_api/ModuleCalibration'

Request:
{   module_calibrate: Int }

Response:
{   success: Boolean,
    message: String, }


```


> Example

```shell

# Refer to rosservice command line API documentation for sample service calls. http://wiki.ros.org/rosservice
    
```

```cpp
//Not Implemented
```

```python
#Not Implemented
```

```cpp--ros
//Not Recommended
```

```python--ros
#Not Recommended
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
success: True
```

```cpp
//Not Implemented
```

```python
#Not Implemented
```

```cpp--ros
//Not Recommended
```

```python--ros
#Not Recommended
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

This API helps calibrate accelerometer, magnetometer, gyroscope, level and RC.

###Parameters:
    
    Following parameters are applicable in RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    module_calibrate | int | module to calibrate.<br>1: accel<br>2: gyro<br>3: mag<br>4: radio<br>7: level
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### ROS endpoint:
APIs in FlytOS are derived from / wrapped around the core services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: Ros Service</br> 
* Name: /\<namespace\>/setup/module_calibration</br>
* Service Type: ModuleCalibration

### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ``POST http://<ip>/ros/<namespace>/setup/module_calibration``
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

* name: '/\<namespace\>/setup/module_calibration'</br>
* serviceType: 'core_api/ModuleCalibration'


### API usage information:
Note: Please take care that accel and mag calibration needs you to rotate the autopilot board in specific direction for the calibration to complete. Refer the calibration widget for the complete procedure.


<aside class="warning">
This API will ONLY work with FlytPOD/PRO and Pixhawk running PX4.
</aside>