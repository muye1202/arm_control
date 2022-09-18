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
    # pen coord: (z, y, x)
    base = np.array([0.05, 0.12, 0.35]) - pen_coord
    base_frame = np.array([base[2], base[1], base[0]])

    # move in xy plane:
    del_y = base_frame[1]
    del_x = base_frame[0]
    # del_y = pen_coord[1] - gripper_pos[1]
    # del_x = abs(gripper_pos[0] - pen_coord[2])
    alpha = np.arctan(del_y / del_x)

    # get current joint pos
    joints_pos = robot.arm.get_joint_commands()
    robot.arm.set_joint_positions([alpha, joints_pos[1], joints_pos[2], joints_pos[3]], moving_time=2, accel_time=2)

    # convert to cam frame
    gripper_pos = base_frame
    
    num_step = 3
    step_x = gripper_pos[0] / num_step
    step_z = 0.6*gripper_pos[2] / num_step
    for _ in range(num_step):
        robot.arm.set_ee_cartesian_trajectory(step_x, 0, step_z, moving_time=1)

    robot.gripper.set_pressure(2.0)
    robot.gripper.grasp()
    time.sleep(1)

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