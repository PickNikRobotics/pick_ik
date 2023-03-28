^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pick_ik
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2023-03-28)
------------------
* Set Werror through CMake presets (`#39 <https://github.com/PickNikRobotics/pick_ik/issues/39>`_)
* Replace lower_bounds with gt_eq (`#37 <https://github.com/PickNikRobotics/pick_ik/issues/37>`_)
* Upgrade with new pkgs to fix issue with ROS
* Target include subdirectory
* Update Catch2 version to 3.3.0
* Fix overriding of package
* Fix orientation calculation in cost function and frame tests (`#31 <https://github.com/PickNikRobotics/pick_ik/issues/31>`_)
  * Fix orientation calculation
  * Update plugin return values
  * Remove redundant (and incorrect) joints bounds check
  * Use Eigen angular distance calculation
* Small grammar fixes (`#28 <https://github.com/PickNikRobotics/pick_ik/issues/28>`_)
* Contributors: Sebastian Castro, Stephanie Eng, Tyler Weaver

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
