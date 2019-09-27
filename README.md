# motion\_plan\_player

This is a ROS package that reads in a 2D matrix from a CSV file and publishes the data as joint states of Franka Emika Panda 7DOF manipulator. 

In CSV file, each row defines a state of 7 joints at a specific time instance:
```
t0_js0,  t0_js1, ... ,t0_js6
t1_js1,  t1_js1, ... ,t1_js6
        . . .
tn_js0,  tn_js1, ... ,tn_js6
```



## Published topics:

  * ~joint\_states (sensor\_msgs/JointState)

## Parameters:

  * ~topic (String, default: "joint\_states")

      Topic on which joint states are published.

  * ~rate (Double, default: 20 Hz)

      The publishing rate (Hz).

  * ~file (String, default: "sample\_data/dataset.csv")

    Name of the CSV file to be loaded.

## Example usage

To run everything with a single command, use the launch file:

```rosrun motion_plan_player motion_plan_player.launch file:=path/to/dataset.csv _rate=10```

To run visualisation and the player node separately use:

```roslaunch motion_plan_player motion_plan_player.launch viz_only:=true```

```rosrun motion_plan_player motion_plan_player _file:=path/to/dataset.csv _rate=10```
