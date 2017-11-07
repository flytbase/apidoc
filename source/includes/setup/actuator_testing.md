# Vehicle Setup APIs
## Actuator Testing

> Definition

```shell
# API call described below requires shell access, either login to the device by connecting a monitor or use ssh for remote login.

ROS-Service Name: /<namespace>/setup/actuator_testing
ROS-Service Type: core_api/ActuatorTesting, below is its description

#request
uint8 actuator_id
float32 time_s
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

URL: 'http://<ip>/ros/<namespace>/setup/actuator_testing'

JSON Request:
{   actuator_id: Int,
    time_s: Float }

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

name: '/<namespace>/setup/actuator_testing',
serviceType: 'core_api/ActuatorTesting'

Request:
{   actuator_id: Int,
    time_s: Float }

Response:
{   success: Boolean,
    message: String
}


```

> Example

```shell
# Refer to rosservice command line API documentation for sample service calls. http://wiki.ros.org/rosservice
```

```cpp
//Not Implemented
```

```python
# Not Implemented
```

```cpp--ros
//Not Recommended
```

```python--ros
# Not Recommended
```

```javascript--REST
var  msgdata={};
msgdata["actuator_id"]=2;
msgdata["time_s"]=4.00;

$.ajax({
    type: "POST",
    dataType: "json",
    data: JSON.stringify(msgdata),
    url: "http://<ip>/ros/<namespace>/setup/actuator_testing",  
    success: function(data){
           console.log(data.success);
           console.log(data.message);
    }
};

```

```javascript--Websocket
var actuatorTesting = new ROSLIB.Service({
    ros : ros,
    name : '/<namespace>/setup/actuator_testing',
    serviceType : 'core_api/ActuatorTesting'
});

var request = new ROSLIB.ServiceRequest({
    actuator_id: 2,
    time_s: 4.00
});

actuatorTesting.callService(request, function(result) {
    console.log('Result for service call on '
      + actuatorTesting.name
      + ': '
      + result.success
      +': '
      + result.message);
});
```

> Example response

```shell
success: true
```

```cpp
//Not Implemented
```

```python
# Not Implemented
```

```cpp--ros
//Not Recommended
```

```python--ros
# Not Recommended
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

### Description:

This API allows for testing an actuator by providing actuator ID and time to rotate as parameters. If the corresponding actuator rotates on execution of the API correctly for the defined time then the motors are correctly connected.

### API usage information:

<aside class="notice">
    Make sure to check the direction of rotation while you trigger this API for correct response from the particular actuator.
</aside>

<aside class="warning">
    This API will **ONLY** work with FlytPOD/PRO and Pixhawk running APM.
</aside>

### Parameters:
    
    Following parameters are applicable for RESTful, Websocket, ROS. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    actuator_id | int | Decide which actuator to trigger.
    time_s | float | Time in seconds to rotate the actuator
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    success | bool | true if action successful
    message | string | debug message

### ROS endpoint:

APIs in FlytOS are derived from / wrapped around the core services in ROS. Onboard service clients in rospy / roscpp can call these APIs. Take a look at roscpp and rospy API definition for message structure. 

* Type: `Ros Service`
* Name: `/<namespace>/setup/actuator_testing`
* Service Type: `ActuatorTesting`

### RESTful endpoint:

FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: `POST http://<ip>/ros/<namespace>/setup/actuator_testing`
* JSON Request:
`{
    actuator_id: Int,
    time_s: Float
}`
* JSON Response:
`{
    success: Boolean
    message: String
}`

### Websocket endpoint:

Websocket APIs can be called from javascript using [roslibjs library](https://github.com/RobotWebTools/roslibjs).

Java websocket clients are supported using [rosjava](http://wiki.ros.org/rosjava).

* name: `/<namespace>/setup/actuator_testing`
* serviceType: `core_api/ActuatorTesting`