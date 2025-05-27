#! /usr/bin/env python2
import cv2
import sys
import numpy as np
#from PIL import Image
import math
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

# Declare a global varialbe for image storage
image = None
br = CvBridge()
scaled_array =[]
scaled_arrayb = Float32MultiArray()
c = 0

# callback function to store the incoming image
def ImageCb(msg):
    global image
    image = br.imgmsg_to_cv2(msg)

    # rospy.loginfo('Image received ...')

def region_of_interest(image):
    pts = np.array([[135,14.5],[152,394],[577,378],[579,5.2]], np.int32)
    pts = pts.reshape((-1,1,2))
    # print(pts[0])
    cv2.polylines(image, [pts], True, (0,255,255),3) # If true the last point is connected to the previous point else if false not connected 
    img_copy = np.copy(image)
    # centroid = np.mean(pts, axis = 0)
    # print(centroid[0][0])
    # print(img_copy)

def cal_dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance

'''
def scale_roi(points, img_copy2):
    scaled_points = []
    mask = np.zeros(img_copy2.shape[:2], dtype=np.uint8)
    # pts = points.reshape((-1,1,2))
    # mask = np.zeros((480, 640), dtype=np.uint8)
    # print("mask shape = ", mask.shape)
    # print("image shape = ", image.shape)
    FILLED = cv2.fillPoly(mask, [points], 255) #points should be int data type
    roi = cv2.bitwise_and(img_copy2, img_copy2, mask=mask)
    # hsvFrame1 = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    non_zero_indices = np.where(mask != 0)
    x_values = non_zero_indices[1]
    y_values = non_zero_indices[0]

    cv2.imshow("ROI: ", roi)
    # cv2.imshow("hsvroi", hsvFrame1)
    # x_min = np.min(points[:, 0])
    # x_max = np.max(points[:, 0])
    # y_min = np.min(points[:, 1])
    # y_max = np.max(points[:, 1])
    # centroid = np.mean(pts, axis = 0)
    centroid = np.mean(points, axis = 0)
    # centroid = np.array([357.0, 210.0])
    print(centroid[0], centroid[1])
    print("max x_values: ", np.max(x_values), "max y_value: ", np.max(y_values))
    # x_scale_values = (x_values - centroid[0])/centroid[0]
    # y_scale_values = (centroid[1] - y_values)/centroid[1]

    x_scale_values = (x_values - centroid[0])/min(centroid[0],(np.max(x_values)-centroid[0]))
    y_scale_values = (centroid[1] - y_values)/min(centroid[1],(np.max(y_values)-centroid[1]))

    print("scaled x values:", x_scale_values)
    print("scaled y values:", y_scale_values)
    print("min x_scaled values: ", np.min(x_scale_values), "max x_scaled values: ", np.max(x_scale_values))
    print("min y_scaled values: ", np.min(y_scale_values), "max y_scaled values: ", np.max(y_scale_values))
    print("start")
    # x_pub.publish(x_min)
    # y_pub.publish(y_max)
    x_pub.publish(np.max(x_scale_values))
    y_pub.publish(np.max(y_scale_values))
    print("stop")        
    # print(x_min, x_max, y_min, y_max)

    ####################################################

    '''

def scale_centre_point(points, centroid, img_copy2, roi_points, object_centers_b):
    # mask = np.zeros(img_copy2.shape[:2], dtype=np.uint8)
    # non_zero_indices = np.where(mask != 0)
    # x_values = non_zero_indices[1]
    # y_values = non_zero_indices[0]
    # print(x_values)
    # roi = cv2.bitwise_and(img_copy2, img_copy2, mask=mask)
    # cv2.imshow("ROI: ", roi)
    scaled_arrayb = Float32MultiArray()

    print("x_point:", points[0][0], "centroid_x: ", centroid[0] )
    x_scale_values = ((points[0][0] - centroid[0])/centroid[0]) #- 0.15
    y_scale_values = ((centroid[1] - points[0][1])/centroid[1]) #+ 0.15
    # x_scale_values = (points[0][0]- centroid[0])/min(centroid[0], (np.max(roi_points)-centroid[0]))
    # y_scale_values = (centroid[1] - points[0][1])/min(centroid[1],(np.max(roi_points)-centroid[1])) 
    
    if x_scale_values >= -0.6 and y_scale_values >= -0.7:
        print("scaled centre values: ", x_scale_values, y_scale_values)
        x_pub.publish(np.max(x_scale_values))
        y_pub.publish(np.max(y_scale_values))
    else:
        print("kein Objekt scaled centre values: ", x_scale_values, y_scale_values)
    
    # centers_msg = ObjectCenters()
    # for center in object_centers_b:
    #     point_msg = Point()
    #     point_msg.x = center[0]
    #     point_msg.y = center[1]
    #     centers_msg.centers.append(point_msg)
    
    # scaled_array_pub.publish(centers_msg)

    # flatten_array = [value for sublist in object_centers_b for value in sublist]
    # scaled_arrayb.data = flatten_array
    # pub.publish(scaled_arrayb)


    # for i, point in enumerate(object_centers_b):
    #     scaled_point = [(point[0]- centroid[0])/centroid[0], (centroid[1] - point[1])/centroid[1] ]
    #     scaled_array.append(scaled_point)
        # print("scaled_array: ", scaled_array)
        # scaled_array_pub = scaled_array
        # flatten_array = [item for sublist in scaled_array for item in sublist]
        # flatten_array = [value for sublist in scaled_array for value in sublist]
        # scaled_arrayb.data = flatten_array
        # print("scaled_array pub: ", scaled_arrayb)
        # pub.publish(scaled_arrayb)
        # rospy.loginfo("Array published: ")
        
    # print("scaled_array: ", scaled_array)
    # print("Scaled array: ", scaled_arrayb)
    # return scaled_array




def Img_process(image):
    img_copy = np.copy(image)
    img_copy2 = np.copy(image)
    hsvFrame = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
    # img_copy = np.copy(image)
    region_of_interest(img_copy)

    ref_pt = [(135,20)]
    img_copy = cv2.circle(img_copy, ref_pt[0], 5, (0,0,255), -1)

    blue_lower = np.array([45, 152, 141])
    blue_upper = np.array([179, 213, 255])
    mask_b = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    kernel = np.ones((5,5), "uint8")
    mask_bb = cv2.dilate(mask_b, kernel)

    centroid = np.array([327.0, 225.0])

    _, contours, _ = cv2.findContours(mask_bb, cv2.RETR_TREE ,cv2.CHAIN_APPROX_SIMPLE)
    # print(contours)

    object_centers_b = [] #For blue objects
    scaled_array = []
    c=0

    if len(contours)!=0:
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            print("area = ", area)
            if(area >= 60):
                x1, y1, x2, y2 = cv2.boundingRect(contour) #coordinates of our object
                # print(x1,y1,x2,y2)
                centre_b = [(int(x1 + (x2/2)), int(y1 + (y2)/2))]
                # centre_b = np.array([[int(x1 + (x2/2)), int(y1 + (y2)/2)]])
                # print(centre_b.shape)
                # print("contour size: ", len(contour))
                M = cv2.moments(contour)
                print("size of M: ", len(M))
                if M["m00"] !=0:
                    center_x = float(M["m10"] / M["m00"])
                    center_y = float(M["m01"] / M["m00"])
                    object_centers_b.append((center_x, center_y))
                    
                    for point in object_centers_b:
                        scaled_x = (point[0] - centroid[0])/centroid[0]
                        scaled_y = (centroid[1] - point[1])/centroid[1]
                        # scaled_point = (scaled_x, scaled_y)
                        # scaled_array.append(scaled_point)
                        scaled_array.append((scaled_x, scaled_y))
                    # x_scale_values = ((object_centers_b[0][0] - centroid[0])/centroid[0]) #- 0.15
                    # y_scale_values = ((centroid[1] - object_centers_b[0][1])/centroid[1]) #+ 0.15
                    # print("Inside M: ", scaled_array)
                    print("Object centres: ", object_centers_b)
                    flatten_array = [value for sublist in object_centers_b for value in sublist]
                    # flatten_array = [(flatten_array[i], flatten_array[i+1]) for i in range(0, len(flatten_array), 2)]
                    # print("result: ", result)
                    print("flatten array: ", flatten_array)
                    scaled_arrayb.data = flatten_array
                    scaled_array_pub.publish(scaled_arrayb)
                    c=c+1
                    print("counter: ", c)
                    rospy.loginfo("Array published: {}".format(scaled_arrayb))

                    # flatten_array = [value for sublist in scaled_array for value in sublist]
                    # print("flatten array: ", flatten_array)
                    # scaled_arrayb.data = flatten_array
                    # scaled_array_pub.publish(scaled_arrayb)
                    # rospy.loginfo("Array published: {}".format(scaled_array))
                print("Inside M: ", scaled_array)

                print("centre_before scale: " , centre_b)
                # dimensions = len(centre_b)
                # print("dimensions: ", dimensions)
                scale_centre_b = [(centre_b[0][0], centre_b[0][1])]
                print("scale_centre_b: ",scale_centre_b)
                # print("x _value: ", scale_centre_b[0][0],"y _value: ", scale_centre_b[0][1])
                img_copy = cv2.rectangle(img_copy, (x1, y1), (x1+x2, y1+y2), (0,0,255), 2)
                cv2.putText(img_copy, "B", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
                distance = cal_dist(centre_b[0], ref_pt[0])
                for point in centre_b:
                    center = [(point[0], point[1])]
                    img_copy = cv2.circle(img_copy, center[0], radius=2, color=(255,0,0), thickness=3)
                    img_copy = cv2.line(img_copy, ref_pt[0], point, (255,0,0), thickness=2)
                    cv2.putText(img_copy, str(int(distance)), (center[0][0], center[0][1] + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
            if(area > 1 and area < 60):
                scale_centre_b = [(0.0 , 0.0)]

    if len(contours) == 0:
        scale_centre_b = [(0.0 , 0.0)]
        print("x _value: ", scale_centre_b[0][0],"y _value: ", scale_centre_b[0][1])

    # new_centre_bx = centre_b[0][0]
    # new_centre_by = centre_b[0][1]
    # scale_centre_b = [(new_centre_bx[0][0], new_centre_by[0][1])]
    # scale_centre_b = [(0.0 , 0.0)]
    roi_points = np.array([[135,14.5],[152,394],[577,378],[579,5.2]], np.int32)
    # centroid = np.mean(roi_points, axis = 0)
    # centroid = np.array([327.0, 225.0])
    # scale_centre_b = np.array([0.5,0.5])
    scale_centre_point(scale_centre_b, centroid, img_copy2, roi_points, object_centers_b)
    # print("scaled_array: ", scaled_array)
    # roi_points = np.array([[135,14.5],[152,394],[577,378],[579,5.2]], np.int32)
    # roi_points = np.array([[135,14.5],[135,394],[579,394],[579,14.5]], np.int32)
    # roi_points = roi_points.reshape((-1,1,2))
    # scaled_roi_points = scale_roi(roi_points)
    # print(scaled_roi_points)

    # scale_roi(roi_points, img_copy2)

    # frame = cv2.circle(frame, (width/2, height/2), 5, (0,255,255), -1)
    image= cv2.circle(img_copy, (357, 210), 5, (0,255,255), -1)
    # print(img_copy)
    cv2.imshow("Stream: ", img_copy)
    # plt.imshow(image)
    # plt.show()
    cv2.imshow("hsv", hsvFrame)
    cv2.imshow('Mask', mask_b) #To track the HSV values of the objects
    # cv2.imshow("Stream: ", hsvFrame)
    cv2.waitKey(1)
    


# the main function
if __name__ == '__main__':
    # 1. Iniitalize a ROS node
    rospy.init_node("image_subscriber_node", anonymous=True)
    # 2. Create a subscriber that subscribes the camera image topic /camera_rect/image_rect
    rospy.Subscriber("/camera_rect/image_rect", Image, ImageCb)
    pub = rospy.Publisher('imagetimer', Image, queue_size=10)

    x_pub = rospy.Publisher('x_topic', Float32, queue_size=10)
    y_pub = rospy.Publisher('y_topic', Float32, queue_size=10)
    scaled_array_pub = rospy.Publisher('scaled_array_topic', Float32MultiArray, queue_size=10)
    # scaled_array_pub = rospy.Publisher('scaled_array_topic', ObjectCenters, queue_size=10)
    # centers_pub = rospy.Publisher('object_centers_topic', ObjectCenters, queue_size=10)
    # 3. Run a loop to access the stored image and apply processing 
    loop_rate = rospy.Rate(30) # 1Hz
    while not rospy.is_shutdown():
        if image is not None:
            # insert image processing code here
            Img_process(image)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                print("break")
                break
            

            pub.publish(br.cv2_to_imgmsg(image))
        loop_rate.sleep()
   
   
   
   






   
   ###########################BELOW IS ALL COMMENTED JUST FOR TRACKING COLOR THRESHOLDVALUES#################################
   
    """
    if len(sys.argv) < 2:
        print("You must give an argument to open a video stream.")
        print("  It can be a number as video device, e.g.: 0 would be /dev/video0")
        print("  It can be a url of a stream,        e.g.: rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov")
        print("  It can be a video file,             e.g.: myvideo.mkv")
        exit(0)

    resource = sys.argv[1]
    # If we are given just a number, interpret it as a video device
    if len(resource) < 3:
        resource_name = "/dev/video" + resource
        resource = int(resource)
        
    else:
        resource_name = resource
    print("Trying to open resource: ", resource_name)
    cap = cv2.VideoCapture(resource)
    if not cap.isOpened():
        print("Error opening resource: " + str(resource))
        print("Maybe opencv VideoCapture can't open it")
        exit(0)

    print("Correctly opened resource, starting to show feed.")
"""
    #####################
    # def track(x):
    #     pass
    #Trackbar
    # cv2.namedWindow('Track')
    # cv2.resizeWindow('Track', 700, 512)
    # cv2.createTrackbar('hue min', 'Track', 45, 179, track)
    # cv2.createTrackbar('hue max', 'Track', 179, 179, track)
    # cv2.createTrackbar('sat min', 'Track', 152, 255, track)
    # cv2.createTrackbar('sat max', 'Track', 213, 255, track)
    # cv2.createTrackbar('val min', 'Track', 141, 255, track)
    # cv2.createTrackbar('val max', 'Track', 255, 255, track)
    #####################
    # roi_x = 135
    # roi_y = 14.5
    # roi_width = 579-135
    # roi_height = 


    # def cal_dist(p1, p2):
    #     x1, y1 = p1
    #     x2, y2 = p2
    #     distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    #     return distance
    
    # def region_of_interest(frame):
    #     pts = np.array([[135,14.5],[152,394],[577,378],[579,5.2]], np.int32)
    #     pts = pts.reshape((-1,1,2))
    #     cv2.polylines(frame, [pts], True, (0,255,255),3)

    ####To get the centre coordinate of our image by LBMOUSECLICK##
    # def mouse_callback(event, x, y, flags, param):
    #     if event == cv2.EVENT_LBUTTONDOWN:
    #         origin_x, origin_y = x, y
    #         print("Origin coordinates:", origin_x, origin_y)
    ###############################################################

    #rval, frame = cap.read()
    # height, width = image.shape[:2]
    # print(width, height)
    # print(height, width)
    # cv2.namedWindow("Frame")
    # cv2.imshow("Frame", frame)
    # cv2.setMouseCallback("Frame", mouse_callback) #To calculate the centre point of our frame

    # while rval:
        # cv2.imshow("Stream: " + resource_name, frame)
        # rval, frame = cap.read()
        # hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # img_copy = np.copy(frame)
        # # cv2.imshow("hsv", hsvFrame)
        # region_of_interest(frame)
        # #################
        # h_min = cv2.getTrackbarPos('hue min', 'Track')
        # h_max = cv2.getTrackbarPos('hue max', 'Track')
        # s_min = cv2.getTrackbarPos('sat min', 'Track')
        # s_max = cv2.getTrackbarPos('sat max', 'Track')
        # v_min = cv2.getTrackbarPos('val min', 'Track')
        # v_max = cv2.getTrackbarPos('val max', 'Track')
        # blue_lower = np.array([h_min, s_min, v_min])
        # blue_upper = np.array([h_max, s_max, v_max])
        # #################
    #     ref_pt = [(135,20)]
    #     frame = cv2.circle(frame, ref_pt[0], 5, (0,0,255), -1)

    #     blue_lower = np.array([45, 152, 141])
    #     blue_upper = np.array([179, 213, 255])
    #     mask_b = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    #     kernel = np.ones((5,5), "uint8")
    #     mask_bb = cv2.dilate(mask_b, kernel)

    #     _, contours, _ = cv2.findContours(mask_bb, cv2.RETR_TREE ,cv2.CHAIN_APPROX_SIMPLE)
    #     for pic, contour in enumerate(contours):
    #         area = cv2.contourArea(contour)
    #         if(area > 50):
    #             x1, y1, x2, y2 = cv2.boundingRect(contour) #coordinates of our object
    #             # print(x1,y1,x2,y2)
    #             centre_b = [(int(x1 + (x2/2)), int(y1 + (y2)/2))]
    #             frame = cv2.rectangle(frame, (x1, y1), (x1+x2, y1+y2), (0,0,255), 2)
    #             cv2.putText(frame, "B", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
    #             distance = cal_dist(centre_b[0], ref_pt[0])
    #             for point in centre_b:
    #                 center = [(point[0], point[1])]
    #                 frame = cv2.circle(frame, center[0], radius=2, color=(255,0,0), thickness=3)
    #                 frame = cv2.line(frame, ref_pt[0], point, (255,0,0), thickness=2)
    #             cv2.putText(frame, str(int(distance)), (center[0][0], center[0][1] + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
        
    #     # frame = cv2.circle(frame, (width/2, height/2), 5, (0,255,255), -1)
    #     frame = cv2.circle(frame, (357, 210), 5, (0,255,255), -1)

    #     cv2.imshow("Stream: " + resource_name, frame)
    #     # plt.imshow(frame)
    #     # plt.show()
    #     # cv2.imshow("hsv", hsvFrame)
    #     cv2.imshow('Mask', mask_b) #To track the HSV values of the objects


    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
    # cap.release()
    # cv2.destroyWindow("preview")
    # cv2.destroyAllWindows()
