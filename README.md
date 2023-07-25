# pick_ik

`pick_ik` is an inverse kinematics (IK) solver compatible with [MoveIt 2](https://github.com/ros-planning/moveit2).

The solver is a reimplementation of [`bio_ik`](https://github.com/TAMS-Group/bio_ik), which combines:
* A local optimizer which solves inverse kinematics via gradient descent
* A global optimizer based on evolutionary algorithms

Critically, `pick_ik` allows you to specify custom cost functions as discussed in  [this paper](https://ieeexplore.ieee.org/document/8460799), so you can prioritize additional objectives than simply solving inverse kinematics at a specific frame. For example, you can minimize joint displacement from the initial guess, enforce that joints are close to a particular pose, or even pass custom cost functions to the plugin.

If you are familiar with `bio_ik`, the functionality in this package includes:
* Reimplementation of the memetic solver (equivalent to `bio1` and `bio2_memetic` solvers)
* Reimplementation of the numeric gradient descent solvers (equivalent to `gd`, `gd_r`, and `gd_c` solvers)
* Fully configurable number of threads if using the global solver
* Cost functions on joint displacement, joint centering, and avoiding joint limits

For more details on the implementation, take a look at the [paper](https://ieeexplore.ieee.org/document/8449979) or the [full thesis](https://d-nb.info/1221720910/34).

---

## Getting Started

To get started using `pick_ik`, refer to the following README files:

* [Installation](doc/INSTALL.md)
* [Usage](doc/USAGE.md)
* [MoveIt Tutorial](https://moveit.picknik.ai/main/doc/how_to_guides/pick_ik/pick_ik_tutorial.html)
