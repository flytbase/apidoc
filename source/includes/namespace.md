# Namespace

> Make sure to replace \<namespace\> with your FlytOS namespace.

```shell
ROS-Service Name: /get_global_namespace
ROS-Service Type: core_api/ParamGetGlobalNamespace, below is its description

#Request : None

#Response : Paramter info
core_api/ParamInfo param_info
#Response : success=true if parameter get was successfull.
bool success
#Response : Returns error message/success message if any.
string message
```

```cpp

```

```python
Class: flyt_python.api.navigation
Function Definition: get_global_namespace()
Arguments: None
return: string
```

```cpp--ros
Function Definition: void ros::param::get("/global_namespace", string global_namespace)
Arguments:
```

```python--ros

```

```shell--curl

```

```javascript--REST

```

```javascript--Webocket

```


> Example API call

```shell
rosservice call /<namespace>/navigation/position_set "twist:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  twist:
    linear: {x: 1.0, y: 3.5, z: -5.0}
    angular: {x: 0.0, y: 0.0, z: 0.5}
tolerance: 0.0
async: false
relative: false
yaw_valid: true
body_frame: false"

#sends (x,y,z)=(1.0,3.5,-5.0)(m), yaw=0.12rad, relative=false, async=false, yaw_valid=true, body_frame=false
#default value of tolerance=1.0m if left at 0    
```

```python

```

```cpp--ros

```

```python--ros

```

```shell--curl

```

```javascript--REST

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

```javascript--REST

```

```javascript--Webocket

```





----------a brief description of the API will come over here---------

This API sends position setpoint command to the autopilot. Additionally, you can send yaw setpoint (yaw_valid flag must be set true) to the vehicle as well. Some abstract features have been added, such as tolerance/acceptance-radius, synchronous/asynchronous mode, sending setpoints relative to current position (relative flag must be set true), sending setpoints relative to current body frame (body_frame flag must be set true).
This command commands the vehicle to go to a specified location and hover. It overrides any previous mission being carried out and starts hovering.

-------rest API doc will be here-------------

Over here we will define the REST endpoint API.