# ros2-humble-franka-mujoco

A Docker image with ROS2 Humble, MoveIT and pymoveit2, [franka_ros2](https://support.franka.de/docs/franka_ros2.html) and [cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros2) for demo development purposes. Uses [pymoveit2](https://github.com/AndrejOrsula/pymoveit2) and some modified configs from [panda_ign_moveit2](https://github.com/AndrejOrsula/panda_ign_moveit2).

## Usage

* **Build** the image with: `build.sh`. This will create image named `ros2-humble-lite`.
* **Run** the image with: `run.sh`. This will open a TMUX session with four shells. You can also use script `run-nohost.sh` - this will run Docker without option `--network=host`. Curiously, it is needed under Windows WSL in order to have container ports visible in the host.

The following ports are exposed:
* 9090 - for Rosbridge
* 8765 - for Foxglove Web Bridge
* 5000 - for demo webapp projects (see dirs under `/share` folder)


The following commands can be useful:

* `~/starttmux.sh` - this is run automatically upon Docker startup. You can customize this script (add more panes, commands run upon startup etc).
* `~/killtmux.sh` - kills all tmux windows (and the Docker container itself)
* `~/panda-rviz.sh` - shows Panda robot in RVIZ2
* `~/panda-movejoint.sh` - a demo script that executes a joint move action through `pymoveit2`. You need to run `~/panda-rviz.sh` first.
* `~/rosbridge.sh` - starts [Rosbridge](https://foxglove.dev/blog/using-rosbridge-with-ros2) on port 9090
* `~/foxglovebridge.sh` - starts [Foxglove Websocket Bridge](https://github.com/foxglove/ros-foxglove-bridge) on port 8765
* `~/foxglove.sh` - starts bundled Foxglove Studio. You need to use either `rosbridge.sh` or `foxglovebridge.sh` in order to communicate with ROS.
