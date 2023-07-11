# UBC Sailbot Boat Simulator

[![Tests](https://github.com/UBCSailbot/boat_simulator/actions/workflows/tests.yml/badge.svg)](https://github.com/UBCSailbot/boat_simulator/actions/workflows/tests.yml)

UBC Sailbot's boat simulator for the new project. This repository contains a
ROS package `boat_simulator`.

## Setup

The boat simulator is meant to be ran inside the [Sailbot Workspace](https://github.com/UBCSailbot/sailbot_workspace)
development environment. Follow the setup instructions for the Sailbot Workspace
[here](https://ubcsailbot.github.io/docs/current/sailbot_workspace/setup/)
to get started and build all the necessary ROS packages.

## Run

The [`launch/`](./launch/) folder contains [ROS 2 launch files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
that each run their own node(s). Run a launch file with:

``` shell
ros2 launch boat_simulator LAUNCH_FILE_NAME
```

## Test

Run the `test` task in the Sailbot Workspace.
