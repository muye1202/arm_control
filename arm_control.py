from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXS
import time
from image_pipeline import get_image, thresholding, centroid,contour, pen_coordinate
import numpy as np
import modern_robotics as mr

# The robot object is what you use to control the robot
robot = InterbotixManipulatorXS("px100", "arm", "gripper")

def grasp_pen():
    # get the position of the pen's centroid:
    color_image, depth_image, depth_scale, intrinsic_param = get_image()
    thresholding(color_image)
    boundary, img = contour()
    cx, cy = centroid(img, boundary)

    # get cx, cy depth from depth image
    centroid_depth = depth_image[cy, cx] * depth_scale
    # pen 3d coordinate
    pen_coord = pen_coordinate(centroid_depth, intrinsic_param, cx, cy)
    
    trials = 0
    alpha = 0
    beta = 0
    gamma = 0 # third angle
    phi = 0 # fourth angle
    while trials < 2:

        # ee pos:
        R, ee_pos = ee_pose()
        # convert to cam frame
        base_frame = pen_coord + np.array([0.15, 0.1, 0])
        gripper_pos = base_frame + ee_pos
        # move in xy plane:
        del_x = pen_coord[0] - gripper_pos[0]
        del_y = pen_coord[1] - gripper_pos[1]
        alpha = np.arctan(del_y / del_x)
        
        robot.arm.set_joint_positions([alpha, beta, 0, 0], moving_time=2, accel_time=2)

        # move in z plane:
        # ee pos:
        R, ee_pos = ee_pose()
        # convert to cam frame
        base_frame = pen_coord + np.array([0.15, 0.1, 0])
        gripper_pos = base_frame + ee_pos

        del_h = pen_coord[2] - gripper_pos[2]
        del_x = pen_coord[0] - gripper_pos[0]
        beta = np.arctan(del_h/del_x)

        robot.arm.set_joint_positions([alpha, beta, 0, 0], moving_time=2, accel_time=2)
            
        trials += 1

    robot.gripper.set_pressure(2.0)
    robot.gripper.grasp()
    time.sleep(2)

    # turn 180 degrees
    turn = 180* np.pi / 180
    #robot.arm.set_joint_positions([-turn, beta, 0, 0], moving_time=4, accel_time=4)
    time.sleep(2)

    robot.arm.go_to_sleep_pose()
    robot.gripper.release()


def ee_pose():
    joints = robot.arm.get_joint_commands()
    T = mr.FKinSpace(robot.arm.robot_des.M, robot.arm.robot_des.Slist, joints)
    [R, p] = mr.TransToRp(T)

    return R, p    


if __name__ == "__main__":
    robot.arm.go_to_sleep_pose()
    robot.gripper.release()
    grasp_pen()


"""
mode = 'h'
# Let the user select the position
while mode != 'q':
    mode=input("[h]ome, [s]leep, [q]uit ")
    if mode == "h":
        robot.arm.go_to_home_pose()
    elif mode == "s":
        robot.arm.go_to_sleep_pose()

print("go to a position: ")
robot.arm.go_to_sleep_pose()
time.sleep(3)
robot.arm.set_joint_positions([0, 0, 0.5, 0], moving_time=2, accel_time=2)
"""