

ROS install: https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html
ROS install Ubuntu: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

### Running the system
Command to run on the Pi to publish camera msgs
```
ros2 run vulnerable_road_users img_publisher
```
Command to run on the laptop to run YOLO on recieved images

```
ros2 run vulnerable_road_users img_subscriber
```

Commands to run on the Pi's for the alerts
```
ros2 run vulnerable_road_users alert_audio
ros2 run vulnerable_road_users alert_light
```
