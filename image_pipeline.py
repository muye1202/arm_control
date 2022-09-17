import pyrealsense2 as rs
import numpy as np
import cv2
from collections import OrderedDict

a = 0
b = 0
c = 0
d = 0
e = 0
f = 0
def change_color_one(x):
    global a
    a = x

def change_color_two(x):
    global b
    b = x

def change_color_three(x):
    global c
    c = x

def change_color_four(x):
    global d
    d = x

def get_image():
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()

    # enable recording and play from file
    # config.enable_record_to_file("real_sense_recording")
    # config.enable_device_from_file("real_sense_recording")

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


    # Start streaming
    profile = pipeline.start(config)
    pro = profile.get_stream(rs.stream.color)       
    intrinsic_param = pro.as_video_stream_profile().get_intrinsics()

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 0.5 #1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Get frameset of color and depth
    frames = pipeline.wait_for_frames()
    # frames.get_depth_frame() is a 640x360 depth image

    # Align the depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
    color_frame = aligned_frames.get_color_frame()

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Remove background - Set pixels further than clipping_distance to grey
    grey_color = 153
    depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
    bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

    # remove noise in image:
    noise_removed = cv2.fastNlMeansDenoisingColored(bg_removed)

    #cv2.imshow('img', color_image)
    #k = cv2.waitKey(1) & 0xFF
    
    return noise_removed, depth_image, depth_scale, intrinsic_param


def thresholding(frame):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # define range of purple color in HSV
    a = 67
    b = 30
    c = 53
    d = 60
    lower = np.array([a-d , b-d , c-d])
    upper = np.array([a+d, b+d, c+d])
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(frame, frame, mask = mask)

    """
    cv2.namedWindow('controls')
    #create trackbar in 'controls' window with name 'r''
    cv2.createTrackbar('first channel','controls',0,360,change_color_one)
    cv2.createTrackbar('second channel','controls',0,360,change_color_two)
    cv2.createTrackbar('third channel','controls',0,360,change_color_three)
    cv2.createTrackbar('fourth channel','controls',0,360,change_color_four)

    while(1):

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        d = 80
        lower = np.array([a-d , b-d , c-d])
        upper = np.array([a+d, b+d, c+d])
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower, upper)
        res = cv2.bitwise_and(frame, frame, mask = mask)

        cv2.imshow("res", res)
        k = cv2.waitKey(1) & 0xFF
    """

    cv2.imwrite('pen_color_image.png', res)

        

def contour():
    img = cv2.imread('pen_color_image.png')
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # imgray = cv2.GaussianBlur(imgray, (7, 7), cv2.BORDER_DEFAULT)
    ret, im = cv2.threshold(imgray, 60, 255, cv2.THRESH_BINARY)

    #cv2.imshow('img after threshold', im)
    #cv2.waitKey(0)

    contours, hierarchy = cv2.findContours(im, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour_dict = {}
    area = []
    for cnt in contours:
        s = cv2.contourArea(cnt)
        area.append(s)
        contour_dict[s] = cnt

    
    pen_cont = []
    area.sort(reverse=True)
    pen_cont.append(contour_dict[area[0]])
    
    
    #cv2.drawContours(img, contours, -1, (0,255,0), 3)
    #cv2.imshow('contours', img)
    #cv2.waitKey(0)

    return pen_cont, img

def centroid(img, contour):
    cx = 0
    cy = 0
    for cnt in contour:
        M = cv2.moments(cnt)
        if M['m00'] != 0:
            cx += int(M['m10']/M['m00'])
            cy += int(M['m01']/M['m00'])

    cx = int(cx / len(contour))
    cy = int(cy / len(contour))

    #cv2.circle(img, (cx, cy), 5, (0,0,255), -1)
    #cv2.imshow('centroid', img)
    #cv2.waitKey(0)

    return cx, cy
    
    """
    cv2.circle(img, (cx, cy), 5, (0,0,255), -1)
    cv2.imshow('centroid', img)
    cv2.waitKey(0)
    """

def pen_coordinate(depth_image, intr, cx, cy):
    coord = rs.rs2_deproject_pixel_to_point(intr, [cx, cy], depth_image)
    return coord

if __name__ == "__main__":
    color_image, depth_image, depth_scale, intrinsic_param = get_image()
    thresholding(color_image)
    #boundary, img = contour()
    #cx, cy = centroid(img, boundary)
    #centroid_depth = depth_image[cx, cy] * depth_scale
    #pen_coord = pen_coordinate(centroid_depth, intrinsic_param, cx, cy)
    #print(pen_coord)
    # cv2.imshow('pen', boundary)

    
