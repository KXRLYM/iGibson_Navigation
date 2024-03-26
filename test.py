import logging 
import os 
import platform
import sys
import time

import numpy as np
import pybullet as p 
import cv2

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from collections import OrderedDict

import igibson
from igibson.robots import REGISTERED_ROBOTS, ManipulationRobot
from igibson.scenes.empty_scene import EmptyScene
from igibson.scenes.gibson_indoor_scene import StaticIndoorScene
from igibson.simulator import Simulator
from igibson.utils.utils import parse_config

from igibson.objects.cube import Cube


ARROWS = {
    0: "up_arrow",
    1: "down_arrow",
    2: "left_arrow",
    3: "right_arrow",
    65295: "left_arrow",
    65296: "right_arrow",
    65297: "up_arrow",
    65298: "down_arrow",
}

gui = "ig"

def get_depth(raw_vision_obs):
    """
    :return: depth sensor reading, normalized to [0.0, 1.0]
    """
    depth = -raw_vision_obs["3d"][:, :, 2:3]
    # 0.0 is a special value for invalid entries
    depth[depth < 0.5] = 0.0
    depth[depth > 5.0] = 0.0

    # re-scale depth to [0.0, 1.0]
    depth /= 5.0
    #depth = self.noise_model.add_noise(depth)

    return depth

class KeyboardController:
    """
    Simple class for controlling iGibson robots using keyboard commands
    """
    def __init__(self, robot, simulator):
        """
        :param robot: BaseRobot, robot to control
        """
        # Store relevant info from robot
        self.simulator = simulator
        self.action_dim = robot.action_dim
        self.controller_info = OrderedDict()
        idx = 0
        for name, controller in robot._controllers.items():
            self.controller_info[name] = {
                "name": type(controller).__name__,
                "start_idx": idx,
                "command_dim": controller.command_dim,
            }
            idx += controller.command_dim

        # Other persistent variables we need to keep track of
        self.joint_control_idx = None  # Indices of joints being directly controlled via joint control
        self.current_joint = -1  # Active joint being controlled for joint control
        self.gripper_direction = 1.0  # Flips between -1 and 1
        self.persistent_gripper_action = None  # Whether gripper actions should persist between commands,
        # i.e.: if using binary gripper control and when no keypress is active, the gripper action should still the last executed gripper action
        self.last_keypress = None  # Last detected keypress
        self.keypress_mapping = None
        self.use_omnidirectional_base = robot.model_name in ["Tiago"]  # add other robots with omnidirectional bases
        self.populate_keypress_mapping()
        self.time_last_keyboard_input = time.time()

    def populate_keypress_mapping(self):
        """
        Populates the mapping @self.keypress_mapping, which maps keypresses to action info:

            keypress:
                idx: <int>
                val: <float>
        """
        self.keypress_mapping = {}
        self.joint_control_idx = set()

        # Add mapping for joint control directions (no index because these are inferred at runtime)
        self.keypress_mapping["]"] = {"idx": None, "val": 0.1}
        self.keypress_mapping["["] = {"idx": None, "val": -0.1}

        # Iterate over all controller info and populate mapping
        for component, info in self.controller_info.items():
            if self.use_omnidirectional_base:
                self.keypress_mapping["i"] = {"idx": 0, "val": 2.0}
                self.keypress_mapping["k"] = {"idx": 0, "val": -2.0}
                self.keypress_mapping["u"] = {"idx": 1, "val": 1.0}
                self.keypress_mapping["o"] = {"idx": 1, "val": -1.0}
                self.keypress_mapping["j"] = {"idx": 2, "val": 1.0}
                self.keypress_mapping["l"] = {"idx": 2, "val": -1.0}
            if info["name"] == "JointController":
                for i in range(info["command_dim"]):
                    ctrl_idx = info["start_idx"] + i
                    self.joint_control_idx.add(ctrl_idx)
            elif info["name"] == "DifferentialDriveController":
                self.keypress_mapping["i"] = {"idx": info["start_idx"] + 0, "val": 0.2}
                self.keypress_mapping["k"] = {"idx": info["start_idx"] + 0, "val": -0.2}
                self.keypress_mapping["l"] = {"idx": info["start_idx"] + 1, "val": 0.1}
                self.keypress_mapping["j"] = {"idx": info["start_idx"] + 1, "val": -0.1}
            elif info["name"] == "InverseKinematicsController":
                self.keypress_mapping["up_arrow"] = {"idx": info["start_idx"] + 0, "val": 0.5}
                self.keypress_mapping["down_arrow"] = {"idx": info["start_idx"] + 0, "val": -0.5}
                self.keypress_mapping["right_arrow"] = {"idx": info["start_idx"] + 1, "val": -0.5}
                self.keypress_mapping["left_arrow"] = {"idx": info["start_idx"] + 1, "val": 0.5}
                self.keypress_mapping["p"] = {"idx": info["start_idx"] + 2, "val": 0.5}
                self.keypress_mapping[";"] = {"idx": info["start_idx"] + 2, "val": -0.5}
                self.keypress_mapping["n"] = {"idx": info["start_idx"] + 3, "val": 0.5}
                self.keypress_mapping["b"] = {"idx": info["start_idx"] + 3, "val": -0.5}
                self.keypress_mapping["o"] = {"idx": info["start_idx"] + 4, "val": 0.5}
                self.keypress_mapping["u"] = {"idx": info["start_idx"] + 4, "val": -0.5}
                self.keypress_mapping["v"] = {"idx": info["start_idx"] + 5, "val": 0.5}
                self.keypress_mapping["c"] = {"idx": info["start_idx"] + 5, "val": -0.5}
            elif info["name"] == "MultiFingerGripperController":
                if info["command_dim"] > 1:
                    for i in range(info["command_dim"]):
                        ctrl_idx = info["start_idx"] + i
                        self.joint_control_idx.add(ctrl_idx)
                else:
                    self.keypress_mapping[" "] = {"idx": info["start_idx"], "val": 1.0}
                    self.persistent_gripper_action = 1.0
            elif info["name"] == "NullGripperController":
                # We won't send actions if using a null gripper controller
                self.keypress_mapping[" "] = {"idx": info["start_idx"], "val": None}
            else:
                raise ValueError("Unknown controller name received: {}".format(info["name"]))

    def get_random_action(self):
        """
        :return Array: Generated random action vector (normalized)
        """
        return np.random.uniform(-1, 1, self.action_dim)

    def get_teleop_action(self):
        """
        :return Array: Generated action vector based on received user inputs from the keyboard
        """
        action = np.zeros(self.action_dim)
        keypress = self.get_keyboard_input()

        if keypress is not None:
            # If the keypress is a number, the user is trying to select a specific joint to control
            if keypress.isnumeric():
                if int(keypress) in self.joint_control_idx:
                    self.current_joint = int(keypress)

            elif keypress in self.keypress_mapping:
                action_info = self.keypress_mapping[keypress]
                idx, val = action_info["idx"], action_info["val"]

                # Non-null gripper
                if val is not None:
                    # If the keypress is a spacebar, this is a gripper action
                    if keypress == " ":
                        # We toggle the gripper direction if the last keypress is DIFFERENT from this keypress AND
                        # we're past the gripper time threshold, to avoid high frequency toggling
                        # i.e.: holding down the spacebar shouldn't result in rapid toggling of the gripper
                        if keypress != self.last_keypress:
                            self.gripper_direction *= -1.0

                        # Modify the gripper value
                        val *= self.gripper_direction
                        if self.persistent_gripper_action is not None:
                            self.persistent_gripper_action = val

                    # If there is no index, the user is controlling a joint with "[" and "]". Set the idx to self.current_joint
                    if idx is None and self.current_joint != -1:
                        idx = self.current_joint

                    if idx is not None:
                        action[idx] = val

            sys.stdout.write("\033[K")
            print("Pressed {}. Action: {}".format(keypress, action))
            sys.stdout.write("\033[F")

        # Update last keypress
        self.last_keypress = keypress

        # Possibly set the persistent gripper action
        if self.persistent_gripper_action is not None and self.keypress_mapping[" "]["val"] is not None:
            action[self.keypress_mapping[" "]["idx"]] = self.persistent_gripper_action

        # Return action
        return action

    def get_keyboard_input(self):
        """
        Checks for newly received user inputs and returns the first received input, if any
        :return None or str: User input in string form. Note that only the characters mentioned in
        @self.print_keyboard_teleop_info are explicitly supported
        """
        global gui

        # Getting current time
        current_time = time.time()
        if gui == "pb":
            kbe = p.getKeyboardEvents()
            # Record the first keypress if any was detected
            keypress = -1 if len(kbe.keys()) == 0 else list(kbe.keys())[0]
        else:
            # Record the last keypress if it's pressed after the last check
            keypress = (
                -1
                if self.simulator.viewer.time_last_pressed_key is None
                or self.simulator.viewer.time_last_pressed_key < self.time_last_keyboard_input
                else self.simulator.viewer.last_pressed_key
            )
        # Updating the time of the last check
        self.time_last_keyboard_input = current_time

        if keypress in ARROWS:
            # Handle special case of arrow keys, which are mapped differently between pybullet and cv2
            keypress = ARROWS[keypress]
        else:
            # Handle general case where a key was actually pressed (value > -1)
            keypress = chr(keypress) if keypress > -1 else None

        return keypress

    @staticmethod
    def print_keyboard_teleop_info():
        """
        Prints out relevant information for teleop controlling a robot
        """

        def print_command(char, info):
            char += " " * (10 - len(char))
            print("{}\t{}".format(char, info))

        print()
        print("*" * 30)
        print("Controlling the Robot Using the Keyboard")
        print("*" * 30)
        print()
        print("Joint Control")
        print_command("0-9", "specify the joint to control")
        print_command("[, ]", "move the joint backwards, forwards, respectively")
        print()
        print("Differential Drive Control")
        print_command("i, k", "turn left, right")
        print_command("l, j", "move forward, backwards")
        print()
        print("Omnidirectional Drive Control")
        print_command("j, l", "turn left, right")
        print_command("i, k", "move forward, backwards")
        print_command("u, o", "move left, right")
        print()
        print("Inverse Kinematics Control")
        print_command("\u2190, \u2192", "translate arm eef along x-axis")
        print_command("\u2191, \u2193", "translate arm eef along y-axis")
        print_command("p, ;", "translate arm eef along z-axis")
        print_command("n, b", "rotate arm eef about x-axis")
        print_command("o, u", "rotate arm eef about y-axis")
        print_command("v, c", "rotate arm eef about z-axis")
        print()
        print("Boolean Gripper Control")
        print_command("space", "toggle gripper (open/close)")
        print()
        print("*" * 30)
        print()


def main():
    """
    Create an iGibson environment from a custom config file and
    launch Fetch for teleop
    """
    raw_modalities = ["rgb", "3d", "seg", "ins_seg", "normal", "optical_flow", "scene_flow"]
    
    # Setting up a simulation, scene and a robot
    s = Simulator(mode = "gui_interactive", use_pb_gui=False, image_height=600, image_width = 1100)

    scene = EmptyScene(floor_plane_rgba=[0.6, 0.6, 0.6, 1])
    s.import_scene(scene)

    fetch_config = parse_config(os.path.join(igibson.configs_path, "robots", "fetch.yaml"))
    robot_config = fetch_config["robot"]
    robot_name = robot_config.pop("name")
    robot= REGISTERED_ROBOTS[robot_name](**robot_config)

    s.import_object(robot)
    robot.set_position([0, 0, 0])

    cube = Cube(pos=[1,5,0], dim=[1,1,1], color=[1,0,0,1])
    s.import_object(cube)

    action_generator = KeyboardController(robot=robot, simulator=s)
    action_generator.print_keyboard_teleop_info()

    max_step = -1
    step = 0
    cv2.namedWindow("Depth Viewer")
    fig = plt.figure()
    ax = Axes3D(fig)
    while step != max_step:
        action = (
            action_generator.get_teleop_action()
        )
        robot.apply_action(action)
        for _ in range(10):
            s.step() # update the position in renderer
            step+=1
        raw_vision_obs = s.renderer.render_robot_cameras(modes=raw_modalities) # you can later do render_active_cameras
        raw_vision_obs = {mode: value for mode, value in zip(raw_modalities, raw_vision_obs)} # processing raw_vision_obs

        # pointcloud rendering found in /sensors/vision_sensor.py 
        pc = raw_vision_obs["3d"][:,:,:3] # returns W * H * 3 channel for colored? I might have to downsample..
        print(pc.shape)
        depth = get_depth(raw_vision_obs) # returns W * H * 1 channel for depth
        pc_reshaped = pc.reshape(pc.shape[0]*pc.shape[1], 3)
        
        plt.show()
        cv2.imshow("Depth Viewer", depth)
        
    
    s.disconnect()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()