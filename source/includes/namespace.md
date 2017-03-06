# Namespace

> Make sure to replace \<ip\> with your FlytOS running device IP.

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




This API gets the namespace, the FlytOS running device is using. This is required for making service calls and REST calls for the all other APIs.

```GET http://< ip >/ros/get_global_namepsace```
