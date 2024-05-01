#!/bin/bash -i

SQUARE_SIZE=0.5
PI_2=1.57079632679

ros2 topic pub --once /move com304_interfaces/msg/Move "{x: $SQUARE_SIZE, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: $PI_2}"
ros2 topic pub --once /move com304_interfaces/msg/Move "{x: $SQUARE_SIZE, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: $PI_2}"
ros2 topic pub --once /move com304_interfaces/msg/Move "{x: $SQUARE_SIZE, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: $PI_2}"
ros2 topic pub --once /move com304_interfaces/msg/Move "{x: $SQUARE_SIZE, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: $PI_2}"

ros2 topic pub --once /move com304_interfaces/msg/Move "{x: $SQUARE_SIZE, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: $PI_2}"
ros2 topic pub --once /move com304_interfaces/msg/Move "{x: $SQUARE_SIZE, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: $PI_2}"
ros2 topic pub --once /move com304_interfaces/msg/Move "{x: $SQUARE_SIZE, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: $PI_2}"
ros2 topic pub --once /move com304_interfaces/msg/Move "{x: $SQUARE_SIZE, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: $PI_2}"

ros2 topic pub --once /move com304_interfaces/msg/Move "{x: $SQUARE_SIZE, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: $PI_2}"
ros2 topic pub --once /move com304_interfaces/msg/Move "{x: $SQUARE_SIZE, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: $PI_2}"
ros2 topic pub --once /move com304_interfaces/msg/Move "{x: $SQUARE_SIZE, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: $PI_2}"
ros2 topic pub --once /move com304_interfaces/msg/Move "{x: $SQUARE_SIZE, y: 0}"
ros2 topic pub --once /rotate com304_interfaces/msg/Rotate "{yaw: $PI_2}"