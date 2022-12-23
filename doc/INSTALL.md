# pick_ik : Installation

## Install from binaries

COMING SOON! Once a release of `pick_ik` is available, you should be able to get this package with, e.g.,:

```
sudo apt install ros-humble-pick-ik
```

---

## Install from source

1. Create a colcon workspace:

```shell
export COLCON_WS=~/ws_moveit2/
mkdir -p $COLCON_WS/src
```

2. Clone this repository in the `src` directory of your workspace.

```shell
cd $COLCON_WS/src
git clone -b main https://github.com/PickNikRobotics/pick_ik.git
```

3. Set up colcon mixins.

```shell
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

4. Build the workspace.

```shell
cd /path/to/your/workspace
colcon build --mixin release
```

---

## Local development in Dev Containers

This repo is also set up for VSCode Dev Containers, so you can develop directly in a Docker container.

1. Install Docker and add yourself to the Docker group.

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

3. Open the project in VSCode and follow the prompts to open the project in a Dev Container.
