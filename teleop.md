# Teleop interface for libsurvive ROS tracker devices

This project publishes tracked object pose and controller input through standard ROS topics. A separate teleop node should subscribe to these topics and map them to robot motion.

## ROS topics exposed by the driver

The ROS driver publishes:

- `/tf` for tracked object poses
- `/joy` for controller button and axis state
- `/velocity` for per-tracker linear/angular velocity
- `/imu` for inertial measurements
- `/cfg` for configuration updates

The relevant code is in `src/component.cpp`:

- TF transform creation: `src/component.cpp` around the pose update event
- Joy message creation: `src/component.cpp` around the button event

## TF frame naming

Each tracked object is published as a TF transform:

```text
libsurvive_world -> <tracker_serial>
```

This is created with:

```yaml
header.frame_id: "libsurvive_world"
child_frame_id: "<tracker_serial>"
```

So a tracker with serial `LHR-123119` appears as:

```text
libsurvive_world -> LHR-123119
```

and another one with serial `LHR-337E983` appears as:

```text
libsurvive_world -> LHR-337E983
```

If your UI labels them as left and right arm devices, map them in your teleop node with a known convention such as:

```text
left_arm_tracker = LHR-123119
right_arm_tracker = LHR-337E983
```

or whatever the actual serial numbers are in the TF tree.

## Joy topic

The driver publishes a standard ROS `sensor_msgs/Joy` message on:

```text
/joy
```

The message fields are:

```yaml
sensor_msgs/Joy:
  header:
    stamp
    frame_id
  axes: []
  buttons: []
```

The driver fills:

```python
joy_msg.header.frame_id = serial_of_tracker
joy_msg.axes = ...
joy_msg.buttons = ...
```

A separate teleop node can subscribe to `/joy` and use:

```python
self.create_subscription(Joy, "/joy", self.joy_cb, 10)
```

with:

```python
def joy_cb(self, msg: Joy):
    print(msg.buttons)
    print(msg.axes)
    print(msg.header.frame_id)
```

## Example TF subscription

```python
from tf2_ros import Buffer, TransformListener
import rclpy
from rclpy.node import Node

class TrackerListener(Node):
    def __init__(self):
        super().__init__('tracker_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.check_transforms)

    def check_transforms(self):
        for tracker in ['LHR-123119', 'LHR-337E983']:
            try:
                t = self.tf_buffer.lookup_transform(
                    'libsurvive_world',
                    tracker,
                    rclpy.time.Time()
                )
                self.get_logger().info(f'{tracker}: {t.transform}')
            except Exception:
                pass
```

## Typical teleop behavior

A separate node should do the following:

1. Subscribe to `/tf` and look up the transform for each tracker serial.
2. Subscribe to `/joy` and read `buttons` and `axes`.
3. Map the left/right poses to robot control logic.
4. Publish robot commands to your target control topic, such as:
   - `/cmd_vel`
   - joint trajectory commands
   - a custom robot teleop topic

## Summary

Another ROS program should listen to:

```text
/tf
```

for transforms such as:

```text
libsurvive_world -> LHR-123119
libsurvive_world -> LHR-337E983
```

and to:

```text
/joy
```

for button and axis state.
