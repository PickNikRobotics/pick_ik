^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pick_ik
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2022-12-08)
------------------
* pick_ik inverse kinematics plugin compatible with MoveIt 2
* Numeric gradient descent (local) solver
* Memetic algorithm (global solver), configurable for single or multi-threading
* Basic goal functions: joint centering, avoid joint limits, minimal joint displacement
* Support for position-only IK and approximate solutions
* Dynamic parameter switching at runtime using `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`
* Docker devcontainer workflow for VSCode
* Contributors: Chris Thrasher, Sebastian Castro, Tyler Weaver
