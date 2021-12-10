# Target Snooping

Nodes to control the PTU (assumed a Camera mounted on top) to look for a target. When the target is in sight, the PTU snooping ends.

## Description

ROS2 Package to snoop for marker.
The package is competed with core and interfaces.
The snooping changes only the pan of the PTU, and the tilt is assumed to be static (parameter, and could be changed via service).

## Target Snooping

The target_snooping node commands the PTU from min_pan to max_pan (with a given number of equidistant steps). If at any time instant, the topic 'target_presence' (type: Bool) has a value True, the Snooping is Successful and the PTU is stopped. If the snooping reaches the pan_max without a positive feedback, the snooping is Failed.

## Params

- Tilt_static: the value of tilt which is used to perform snooping.
- Discretization: how many steps to make during snooping (equally distributed between pan limits).
- time_to_sleep: how many seconds to remove in one position (to wait for target presence).

# Limits

If mode is auto, the limits are queried from HAL PTU node. if mode is manual, the user can specify min_pan and max_pan explicitly.

# Target Type

If target_type is aruco, then the user must specify marker_in_sight_prefix and marker_id. The target_presence_topic will be marker_in_sight_prefix + marker_id.
in any other case, the target presence topic is specified directly by subscribers.target_presence param.

## Depend

- ROS2
- ptu_interfaces
