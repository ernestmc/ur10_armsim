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

## Joint space target

You can send a target for the joint angles and the robot will move until it reaches the position.
An action server is implemented for this purpose under the `/joint_target` namespace. To send the goal do as the following example:
```
rostopic pub -1 /joint_target/goal ur10_armsim/JointTargetActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  target_angles: [0.0, -2.0, 0.8, 0.0, 0.0, 0.0]" 

```

The position of the TCP will be published as a feedback message and also under the `/tcp_pose` and showed in Rviz.
When the move finishes, a result message will be published under the `/joint_target/result` topic indicating the final angular positions reached.


## Flying ball

A flying ball is simulated and showed in Rviz. The ball has it's own action node whiche you can use to send start and end points and the speed for the movement. For example:
```
rostopic pub -1 /ball/goal ur10_armsim/FlyBallActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  start:
    x: 0.0
    y: 0.0
    z: 0.0
  end:
    x: -1.0
    y: 0.0
    z: 0.3
  speed: 0.3" 
```
You can see the ball position in the tcp frame and the distance to the tcp in the feedback topic while the ball is moving.
The position of the ball in world frame is published in the `/ball/world_pos` topic.
