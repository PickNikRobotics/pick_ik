# pick_ik : Usage

## Using pick_ik as a Kinematics Plugin

As discussed in the MoveIt2 documentation, you can use the [MoveIt Setup Assistant](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html) or change the [`kinematics.yaml` file](https://moveit.picknik.ai/main/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html?highlight=kinematics%20yaml#kinematics-configuration) for your robot setup to use `pick_ik` as the IK solver.

An example `kinematics.yaml` file might look as follows:

```yaml
panda_arm:
  kinematics_solver: pick_ik/PickIkPlugin
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3
  mode: global
  rotation_scale: 0.5
  twist_threshold: 0.001
  cost_threshold: 0.001
  minimal_displacement_weight: 0.0
  gd_step_size: 0.0001
```

As a sanity check, you could follow the [MoveIt Quickstart in RViz](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) tutorial and change the `moveit_resources/panda_moveit_config/config/kinematics.yaml/kinematics.yaml` file to use a configuration like the one above.

---

## Parameter Description

For an exhaustive list of parameters, refer to the [parameters YAML file](../src/pick_ik_parameters.yaml).

Some key parameters you may want to start with are:

* `mode`: If you choose `local`, this solver will only do local gradient descent; If you choose `global`, it will also enable the evolutionary algorithm. Using the global solver will be less performant, but if you're having trouble getting out of local minima, this could help you. We recommend using `local` for things like relative motion / Cartesian interpolation / endpoint jogging, and `global` if you need to solve for goals with a far-away initial conditions.
* `memetic_<property>`: All the properties that only kick in if you use the `global` solver. The key one is `memetic_num_threads`, as we have enabled the evolutionary algorithm to solve on multiple threads.
* `cost_threshold`: This solver works by setting up cost functions based on how far away your pose is, how much your joints move relative to the initial guess, and custom cost functions you can add. Optimization succeeds only if the cost is less than `cost_threshold`. Note that if you're adding custom cost functions, you may want to set this threshold fairly high and rely on `twist_threshold` to be your deciding factor whereas this is more of a guideline.
* `twist_threshold`: Optimization succeeds only if the pose difference is less than `twist_threshold`. So, e.g., assuming your orientation was perfectly met, a `twist_threshold` of 0.001 would mean a 1 mm accuracy.
* `rotation_scale`: If you want position-only IK, set this to 0.0. If you want to treat position and orientation equally, set this to 1.0. You can also use any value in between; it's part of the cost function. Note that the exit condition on `twist_threshold` will ignore orientation if you use `rotation_scale = 0.0`.
* `minimal_displacement_weight`: This is one of the standard cost functions that checks for the joint angle difference between the initial guess and the solution. If you're solving for far-away goals, leave it to zero or it will hike up your cost function for no reason. Have this to a small non-zero value (e.g., 0.001) if you're doing things like Cartesian interpolation along a path, or endpoint jogging for servoing.

You can test out this solver live in RViz, as this plugin is programmed to respond to parameter changes at every solve. This means that you can change values on the fly using the ROS 2 command-line interface, e.g.,

```shell
ros2 param set /rviz2 robot_description_kinematics.panda_arm.mode global

ros2 param set /rviz2 robot_description_kinematics.panda_arm.minimal_displacement_weight 0.001
```

---

## Custom Cost Functions

The [kinematics plugin](../src/pick_ik_plugin.cpp) allows you to pass in an additional argument of type `IkCostFn`, which can be passed in from common entrypoints such as `RobotState::setFromIK()`. See [this page](https://moveit.picknik.ai/humble/doc/examples/robot_model_and_robot_state/robot_model_and_robot_state_tutorial.html?highlight=setfromik#inverse-kinematics) for a usage example.

Alternatively, feel free to add your own cost functions to the `pick_ik` source code itself (specifically, in [`goal.hpp`](../include/goal.hpp) and [`goal.cpp`](../src/goal.cpp) and submit a pull request with the new functionality you add.