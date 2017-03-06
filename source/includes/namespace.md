# Namespace

> Make sure to replace \<ip\> with your FlytOS running device IP.

```shell
ROS-Service Name: /<namespace>/navigation/position_set
ROS-Service Type: core_api/PositionSet, below is its description

#Request : expects position setpoint via twist.twist.linear.x,linear.y,linear.z
#Request : expects yaw setpoint via twist.twist.angular.z (send yaw_valid=true)
geometry_msgs/TwistStamped twist
float32 tolerance
bool async
bool relative
bool yaw_valid
bool body_frame

#Response : success=true - (if async=false && if setpoint reached before timeout = 30sec) || (if async=true)
bool success
```

```cpp
Function Definition: int Navigation::position_set(float x, float y, float z, float yaw=0, float tolerance=0, bool relative=false, bool async=false, bool yaw_valid=false, bool body_frame=false)
Arguments:
	:param x,y,z: Position Setpoint in NED-Frame (in body-frame if body_frame=true)
	:param yaw: Yaw Setpoint in radians
	:param yaw_valid: Must be set to true, if yaw setpoint is provided
	:param tolerance: Acceptance radius in meters, default value=1.0m
	:param relative: If true, position setpoints relative to current position is sent
	:param async: If true, asynchronous mode is set
	:param body_frame: If true, position setpoints are relative with respect to body frame
	:return: For async=true, returns 0 if the command is successfully sent to the vehicle, else returns 1. For async=false, returns 0 if the vehicle reaches given setpoint before timeout=30secs, else returns 1.
```

```python
Class: flyt_python.api.navigation
Function Definition: get_global_namespace()
Arguments: None
return: string
```

```cpp--ros

```

```python--ros

```

```shell--curl

```

```javascript
URL: ' <ip>/ros/get_global_namespace'

JSON Response:
	{
		success: Boolean
		param_info:{
			param_value: String
		}
	}

```

```java

```


> Example API call

```shell

```

```python

```

```cpp--ros

```

```python--ros

```

```shell--curl

```

```javascript
	$.ajax({
	    type: "GET",
	    dataType: "json",
	    url: "http://<ip>/ros/get_global_namespace",   
	    success: function(data){
	        console.log(data);
	    }
	});


```

```javascript--Webocket

```


> Example API Response

```shell
success: true
```

```python

```

```cpp--ros

```

```python--ros

```

```shell--curl

```

```javascript
	data:{
		success:True,
		param_info:{
			param_value:'flytpod'
		}
	}

```

```java

```




This API gets the namespace, the FlytOS running device is using. This is required for making python, cpp service calls and REST calls for the all other APIs.

```GET http://< ip >/ros/get_global_namepsace```