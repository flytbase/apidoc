---
title: FlytAPI Reference

language_tabs:
  - cpp: C++
  - python: Python
  - javascript--REST: JS REST
  - javascript--Websocket: JS Websocket
  - shell: ros-cli
  - cpp--ros: roscpp
  - python--ros: rospy
  - python--flyt_python: flyt_python


includes:
  - namespace
  - navigation/access_request
  - navigation/arm
  - navigation/disarm
  - navigation/takeoff
  - navigation/land
  - navigation/position_hold
  - navigation/position_setpoint
  - navigation/position_setpoint_global
  - navigation/velocity_setpoint
  - navigation/execute_script
  - navigation/get_waypoints
  - navigation/set_waypoints
  - navigation/execute_waypoints
  - navigation/clear_waypoints
  - navigation/pause_waypoints
  - navigation/set_current_waypoint
  - navigation/set_home
  - navigation/rtl
  - telemetry/get_attitude_quat
  - telemetry/get_attitude_euler
  - telemetry/get_local_position
  - telemetry/get_global_position
  - telemetry/get_battery_status
  - telemetry/get_hud
  - telemetry/get_RC
  - telemetry/get_distance_sensor
  - telemetry/get_vehicle_state
  - payload/get_adc
  - payload/gimbal_control
  - payload/obstacle_detection
  - payload/collision_avoidance
  - setup/actuator_testing
  - setup/esc_calibration
  - setup/module_calibration
  - parameter/param_set
  - parameter/param_get_all
  - parameter/param_get
  - parameter/param_save
  - parameter/param_load
  - parameter/param_create
  - parameter/param_delete
  - parameter/param_reset
  - video_streaming
  
toc_footers:
  - <a href='https://flytbase.com'>FlytBase</a>
  - <a href='http://forums.flytbase.com'>Developer Forums</a>
  - <a href='http://docs.flytbase.com'>Documentation</a>
  - <a href='https://github.com/flytbase'>Github Organization</a>

search: true

---

# Introduction

Welcome to API reference documentation for FlytOS. Here you can find details of all the FlytAPIs with their description, parameters and usage examples. API bindings are available in several languages and you can select the desired language from the tabs in the right panel.

If you are **FlytCloud customer**, base IP/URL for API is ```https://dev.flytbase.com```

To learn more about how to get started with FlytOS and build your first app, please visit the developer <a href='http://docs.flytbase.com/'>documentation</a>.

In case, you find any bug/issue in this documentation, please help us rectify it by raising an issue in our <a href='https://github.com/flytbase/flytdocs-slate'>github repository</a>.
