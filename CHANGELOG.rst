^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pick_ik
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2023-12-13)
------------------
* Support continuous (unbounded) joints properly (`#59 <https://github.com/PickNikRobotics/pick_ik/pull/59>`_)
* Run elite gradient descent in separate threads (`#61 <https://github.com/PickNikRobotics/pick_ik/pull/61>`_)
* Contributors: Sebastian Castro

1.0.2 (2023-07-25)
------------------
* New options to control solution quality and performance
   * Option to rerun optimization until valid solution or timeout (`#53 <https://github.com/PickNikRobotics/pick_ik/pull/53>`_)
   * Option to keep optimizing after a valid solution was found (`#46 <https://github.com/PickNikRobotics/pick_ik/pull/46>`_)
   * Approximate solution cost threshold (`#51 <https://github.com/PickNikRobotics/pick_ik/pull/51>`_) and joint jump threshold parameters (`#42 <https://github.com/PickNikRobotics/pick_ik/pull/42>`_)
   * Position scale parameter (`#47 <https://github.com/PickNikRobotics/pick_ik/pull/47>`_)
* Fix ordering of joint limits when loading robot model (`#54 <https://github.com/PickNikRobotics/pick_ik/pull/54>`_)
* Fix use of solution callback (`#48 <https://github.com/PickNikRobotics/pick_ik/pull/48>`_)
* Remove unnecessary preprocessor macro (`#40 <https://github.com/PickNikRobotics/pick_ik/pull/40>`_)
* Contributors: Marc Bestmann, Sebastian Castro, Erik Holum

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
* Dynamic parameter switching at runtime using `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_
* Docker devcontainer workflow for VSCode
* Contributors: Chris Thrasher, Sebastian Castro, Tyler Weaver
