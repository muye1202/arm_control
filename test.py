from sqlite3 import Row
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXS
import time
from image_pipeline import get_image, thresholding, centroid,contour, pen_coordinate
import numpy as np
import modern_robotics as mr

robot = InterbotixManipulatorXS("px100", "arm", "gripper")
if __name__ == "__main__":
    
    robot.arm.set_ee_cartesian_trajectory(x = 0.1, y =0, z =0.1, moving_time=2)
    joints = robot.arm.get_joint_commands()
    T = mr.FKinSpace(robot.arm.robot_des.M, robot.arm.robot_des.Slist, joints)
    [R, p] = mr.TransToRp(T)
    print(p)
    robot.arm.go_to_sleep_pose()
