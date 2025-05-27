# Launch File Usage

Use this command to launch the system and specify your UR robot type and IP address:

```bash
ros2 launch bringup project.launch.py ur_type:=ur10e robot_ip:=143.25.150.2
```


You can change the `ur_type` and `robot_ip` values according to your setup.

---

## Default Configuration

The launch file is set up for the **UR3 robot nicknamed "Eevee"** by default.  
If you are using this robot, you can simply run:

```bash
ros2 launch bringup project.launch.py
```

No need to specify extra arguments in this case.

---

# Move to Home Position

To move the robot to its home position, use the following command:

```bash
ros2 topic pub /go_home std_msgs/msg/String "{data: 'go'}" --once
```

Make sure the robot controller is running before using this command. You can use the command at any time to return the robot to its home position, regardless of its current activity.

---