"""
Image processing unit that outputs the centroid of the pen.

This pipeline first gets image input from Realsense cam,
applies background removal and noise reduction, extracts
contour of the pen to grasp, and lastly calculates the
centroid of the pen.
"""
import pyrealsense2 as rs
import numpy as np
import cv2

a = 0
b = 0
c = 0
d = 0
edge_low = 0
edge_upper = 0

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
    
    return noise_removed, depth_image, depth_scale, intrinsic_param

def color_calibration(frame):
    """
    Allow the user to calibrate the color mask.

    The user can calibrate the color mask so only
    the desired object will be visible in the image,
    whose contour can then be extracted.

    Input:
        frame: Image frame from get_image().
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
        lower = np.array([a-d , b-d , c-d])
        upper = np.array([a+d, b+d, c+d])
        
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower, upper)
        res = cv2.bitwise_and(frame, frame, mask = mask)

        cv2.imshow("res", res)
        _ = cv2.waitKey(1) & 0xFF

def thresholding(frame):
    """
    Make only the pen visible in the image.

    Input:
        frame: Image frame from get_image().
    """
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # define range of purple color in HSV
    a = 114
    b = 32
    c = 30
    d = 48
    lower = np.array([a-d , b-d , c-d])
    upper = np.array([a+d, b+d, c+d])
    
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(frame, frame, mask = mask)

    cv2.imwrite('pen_color_image.png', res)

def edge_lower(x):
    global edge_low
    edge_low = x

def edge_up(x):
    global edge_upper
    edge_upper = x

def edge_cal():
    """
    Extract the edges of the desired object.
    """
    img = cv2.imread('pen_color_image.png')
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    cv2.namedWindow('controls')
    #create trackbar in 'controls' window with name 'r''
    cv2.createTrackbar('first channel','controls',0,360,edge_lower)
    cv2.createTrackbar('second channel','controls',0,360,edge_up)

    while(1):
        edges = cv2.Canny(imgray, edge_low, edge_upper)
        cv2.imshow('edges', edges)

        _ = cv2.waitKey(1) & 0xFF

def contour():
    """
    Extract the contours of the object in an image frame.
    """
    img = cv2.imread('pen_color_image.png')
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(imgray, 51, 109)
    _, im = cv2.threshold(edges, 60, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(im, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # concatenate the contours:
    total_cont = contours[0]
    for cnt in contours[1:]:
        total_cont = np.concatenate((total_cont, cnt), axis=0)
        
    cv2.drawContours(img, total_cont, -1, (0,255,0), 3)
    cv2.imwrite('contour.png', img)

    return total_cont, img

def centroid(img, contour):
    """
    Calculate the centroid of the contour in image.
    """
    mmt = cv2.moments(contour)
    cx = int(mmt['m10']/mmt['m00'])
    cy = int(mmt['m01']/mmt['m00'])

    cv2.circle(img, (cx, cy), 5, (0,0,255), -1)
    cv2.imwrite('centroid.png', img)

    return cx, cy

def pen_coordinate(depth_image, intr, cx, cy):
    coord = rs.rs2_deproject_pixel_to_point(intr, [cy, cx], depth_image)
    return coord

if __name__ == "__main__":
    color_image, depth_image, depth_scale, intrinsic_param = get_image()
    color_calibration(color_image)

