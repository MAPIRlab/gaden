# Gaden common package

This package serves as a wrapper around the `gaden_core` library, so the rest of the gaden packages can just include this one through the usual ament mechanisms.

It also (optionally, see CMakeLists) compiles the `gaden_gui` executable and installs it as if it were a ROS node (even though it's really not). This means you can run the GUI with

```
ros2 run gaden_common gaden_gui
```