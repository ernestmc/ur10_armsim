# Simple robot arm simulator

This package simulates a Universal Robot UR10 robot arm.

## Configuration

The simulation parameters can be configured in the following file `config/ur10.yaml`.
Available parameters are the link names, Denavit-Hartenberg parameters defining each link, maximum speed for each joint and initial joint angle configuration.

## Launching the simulation

To launch the simulator:
```
roslaunch ur10_armsim ur10_armsim.launch
```

This will bring up the simulator node and Rviz to view the position of the joints.

