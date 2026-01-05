# rlc_controller_cpp

This package provides:

- A legacy ROS 2 node controller (`controller_node_cpp`) (optional build)
- A `ros2_control` controller plugin: `rlc_controller_cpp/RealtimeEffortController`

## ros2_control plugin usage

In your controller manager YAML:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000
    rlc_effort_controller:
      type: rlc_controller_cpp/RealtimeEffortController

rlc_effort_controller:
  ros__parameters:
    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7]
```

Then load/activate with the spawner:

```bash
ros2 run controller_manager spawner rlc_effort_controller
```

Note: the hardware must expose `<joint>/position` and `<joint>/velocity` state interfaces, and
`<joint>/effort` command interfaces for the specified joints.

