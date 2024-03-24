import logging 
import os 
import platform
import sys
import time

import numpy as np
import pybullet as p 

import igibson
from igibson.robots import REGISTERED_ROBOTS, ManipulationRobot
from igibson.scenes.empty_scene import EmptyScene
from igibson.scenes.gibson_indoor_scene import StaticIndoorScene
from igibson.simulator import Simulator
from igibson.utils.utils import parse_config

def main():
    """
    Create an iGibson environment from a custom config file and
    launch Fetch for teleop
    """

    s = Simulator(mode = "gui_interactive", use_pb_gui=False, image_height=600, image_width = 1100)

    scene = EmptyScene(floor_plane_rgba=[0.6, 0.6, 0.6, 1])
    s.import_scene(scene)

    fetch_config = parse_config(os.path.join(igibson.configs_path, "robots", "fetch.py"))
    robot_config = fetch_config["robot"]
    robot_name = robot_config.pop("name")
    robot= REGISTERED_ROBOTS[robot_name](**robot_config)

    s.import_object(robot)
    robot.set_position([0, 0, 0])