# Target Snooping

Nodes to control the PTU (assumed a Camera mounted on top) to look for a target.

## Description

ROS2 Package to snoop for marker.
The package is competed with core and interfaces.

## Target Snooping

- ptu_controller: to control actively the ptu
- camera_to_ptu_base: to compute the transformation of the target from camera_link to ptu_base link (based on the status of the PTU). Computation needed by ptu controller

Other tests for unit performance evaluations (max curvature tuning).

## Depend

- ROS2
- flir_ptu_d46_interfaces
