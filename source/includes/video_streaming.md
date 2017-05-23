# Video Streaming API


> Definition

```shell

```

```cpp

```

```python
# Python API described below can be used in onboard scripts only. For remote scripts you can use http client libraries to call FlytOS REST endpoints from Python.

Not Implemented
```

```cpp--ros

```

```python--ros


```

```javascript--REST
This is a REST call for the API. Make sure to replace 
    ip: ip of the FlytOS running device
    namespace: namespace used by the FlytOS device.

URL: 'http://<ip>/list_streams'

JSON Request:
{   namespace: String}

JSON Response:
{   stream1: String,
    stream2: String,.... }

```

```javascript--Websocket
NA


```


> Example

```shell
Not Implemented
```

```cpp
Not Implemented
```

```python
Not Implemented

```

```cpp--ros
Not Implemented
```

```python--ros
Not Implemented
```

```javascript--REST
var msgdata=[];
msgdata['namespace']='flytpod';
// Use getnamespace API to know the namespace

$.ajax({
    type: "POST",
    dataType: "json",
    url: "http://<ip>/list_streams",  
    success: function(data){
           console.log(data['stream1']+" "+data['stream2']);
    }
};

```

```javascript--Websocket
NA
```


> Example response

```shell
Not Implemented
```

```cpp
Not Implemented
```

```python
Not Implemented
```

```cpp--ros
Not Implemented
```

```python--ros
Not Implemented
```

```javascript--REST
{
    stream1: <link to stream1>,
    stream2: <link to stream2>,
    stream3: <link to stream3>,
    stream4: <link to stream4>,
    stream5: <link to stream5>,
    .
    .
    .    
}

```

```javascript--Websocket
NA
```





###Description:
This API allows to get the list of video streams avalibale and view the live stream.

###Parameters:
    
    Following parameters are applicable in RESTful, Websocket. However the description of these parameters applies to all platforms. 
    
    Arguments:
    
    Argument | Type | Description
    -------------- | -------------- | --------------
    namespace | string | namespace used in FlytOS   
    
    Output:
    
    Parameter | Type | Description
    ---------- | ---------- | ------------
    stream1 | string | link of the video stream1
    stream2 | string | link of the video stream2
    stream3 | string | link of the video stream3
    .
    .
    .


### RESTful endpoint:
FlytOS hosts a RESTful server which listens on port 80. RESTful APIs can be called from remote platform of your choice.

* URL: ``POST http://<ip>/list_streams``
* JSON Request:
{   namespace: \<namespace of FlytOS\>}

* JSON Response:
{
    stream1: \<link to stream1\>,
    stream2: \<link to stream2\>,
    stream3: \<link to stream3\>,
    stream4: \<link to stream4\>,
    stream5: \<link to stream5\>,
    .
    .
    .    
}



### API usage information:
Note: To view the video of a particular stream from the list of streams you need to create an **img** tag add the link to its source.

``<img src='http://<ip>/stream?topic=<link to stream1>' > ``

Tip: You can add the following parameters as query string to the link for lighter or better resolution video quality.
width, height, quality, rate:1/2/3

rate:1 will send out every frame, 2 will send out every second frame, 3 every third and so on..

Tip: To stop the video stream you need to delete the **img** tag completely.

Tip: To take a snapshot of the stream replace the word stream with snapshot in the link.

``<img src='http://<ip>/snapshot?topic=<link to stream1>' >``


Note: **Keep an eye out, for this API needs a port at the end of the IP set to :8080.**