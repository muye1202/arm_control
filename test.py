from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXS
import time
from image_pipeline import get_image, thresholding, centroid,contour, pen_coordinate
import numpy as np
import modern_robotics as mr

robot = InterbotixManipulatorXS("px100", "arm", "gripper")
if __name__ == "__main__":
    robot.arm.go_to_home_pose()
    robot.arm.set_joint_positions([0,0.5,0,0], moving_time=2, accel_time=2)
    robot.arm.go_to_sleep_pose()
