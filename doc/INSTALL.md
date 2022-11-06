# pick_ik : Installation

## Install from binaries

COMING SOON! Once a release of `pick_ik` is available, you should be able to get this package with, e.g.,:

```
sudo apt install ros-humble-pick-ik
```

---

## Install from source

1. Clone this repository in a Colcon workspace.

```shell
cd /path/to/your/workspace/src
git clone -b main https://github.com/PickNikRobotics/pick_ik.git
```

2. Import dependencies.

```shell
sudo apt install python3-vcstool
vcs import src < upstream.repos
```

3. Set up colcon mixins.

```shell
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

4. Build the workspace.

NOTE: For now, there is no binary release for [RSL](https://github.com/PickNikRobotics/RSL), so you must add the `shared` Colcon mixin as follows:

```shell
cd /path/to/your/workspace
colcon build --mixin release shared
```

---

## Local development in devcontainer

This repo is also set up for VSCode dev containers, so you can develop directly in a Docker container.

1. Install docker and add yourself to the docker group.

```shell
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

2. Run these commands to create a directory to mount for ccache and another to mount for the ros directory containing log files.

```bash
mkdir -p ~/.local/.pick_ik/ccache
mkdir -p ~/.local/.pick_ik/ros
```

3. Open project in VSCode and follow prompts to open project in devcontainer.
