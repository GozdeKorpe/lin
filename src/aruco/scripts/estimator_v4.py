#!/usr/bin/env python3
# ----------------------------------------------------------------------------------
# Aruco Marker Based Robot Pose Estimation Project - Main Function
# Created on : 26.10.2022
# Author     : Gözde Körpe
# Version    : 1.0
# ----------------------------------------------------------------------------------

import numpy as np
import cv2
import cv2.aruco as aruco
import math
import logging
import matplotlib.pyplot as plt
from aruco_ids import *
import rospy

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
# ----------------------------------------------------------------------------------
# MAIN PARAMETERS
# ----------------------------------------------------------------------------------
MARKER_SIZE = 10  # - [cm]
RECORD = True
x_position = []
y_position = []
yaw_angle = []

marker_positions = [id_72, id_75,id_97]
str_marker_positions = ["id_00", "id_01", "id_02", "id_03", "id_08", "id_09", "id_10", "id_11", "id_12", "id_13", "id_14", "id_15", "id_16", "id_72",
                        "id_17", "id_18", "id_19", "id_20", "id_21", "id_22", "id_23", "id_24", "id_25", "id_26", "id_27", "id_28", "id_29", "id_97"]

# ------ Position DataS Logging Initializer --------------

logger_file_name = "newfile.log"
# Create and configure logger
logging.basicConfig(filename=logger_file_name,
                    format='%(asctime)s %(message)s',
                    filemode='w')
# Creating an object
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
# ------------------------------------------------------------------------------
# ------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
# ------------------------------------------------------------------------------


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


# --- Get the camera calibration path
camera_matrix = np.loadtxt('cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt('cameraDistortion.txt', delimiter=',')

# --- 180 deg rotation matrix around the x axis
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

# --- Define the aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()


def publish_message():
    pub= rospy.Publisher('/position', PoseStamped, queue_size=1)
   
    # initialize the publishing node
    rospy.init_node('cam_position', anonymous=True)

    # define how many times per second
    # will the data be published
    # let's say 10 times/second or 10Hz
    rate = rospy.Rate(60)

    # --- Capture the video camera
    cap = cv2.VideoCapture(0)

    # -- Set the camera size as the one it was calibrated with
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # -- Font for the text in the image
    font = cv2.FONT_HERSHEY_PLAIN

    one_shot = True
    while not rospy.is_shutdown():
        # -- Read the camera frame
        ret, frame = cap.read()

        # -- Convert in gray scale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # -- remember, OpenCV stores color images in Blue, Green, Red

        # -- Find all the aruco markers in the image
        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                                                     cameraMatrix=camera_matrix, distCoeff=camera_distortion)

        if ids is not None:
            value = min(ids[:])[0]
            min_id_marker_index = np.where(ids == value)[0][0]
            ret = aruco.estimatePoseSingleMarkers(corners[min_id_marker_index], MARKER_SIZE,
                                                  camera_matrix, camera_distortion)

            # -- Unpack the output, get only the first
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

            # -- Draw the detected marker and put a reference frame over it
            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

            # -- Print the tag position in camera frame
            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f" % (tvec[0], tvec[1], tvec[2])
            # cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # -- Obtain the rotation matrix tag->camera
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T

            # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip * R_tc)

            # -- Print the marker's attitude respect to camera frame
            str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (math.degrees(roll_marker),
                                                                          math.degrees(pitch_marker),
                                                                          math.degrees(yaw_marker))
            # cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # -- Now get Position and attitude for the camera respect to the marker
            pos_camera = -R_tc * np.matrix(tvec).T

            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (pos_camera[0], pos_camera[1], pos_camera[2])
            cv2.putText(frame, str_position, (0, 20), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # -- Get the attitude of the camera respect to the frame
            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (math.degrees(roll_camera),
                                                                          math.degrees(pitch_camera),
                                                                          math.degrees(yaw_camera))

            cv2.putText(frame, str_attitude, (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            for item in str_marker_positions:
                if value < 10:
                    str_value = "0"+str(value)
                else:
                    str_value = str(value)
                if str_value in item:
                    position_index = str_marker_positions.index(item)

            try:
                absolute_y_position = marker_positions[position_index]['y_position'] + pos_camera[1]
                absolute_x_position = marker_positions[position_index]['x_position'] + pos_camera[0]
                x_position.append(float(absolute_x_position))
                y_position.append(float(absolute_y_position))
                yaw_angle.append(float(yaw_camera))
                message = f"x position: {absolute_y_position},y position: {absolute_x_position}, yaw angle {yaw_angle}"
                logger.info(message)
                				
                p = PoseStamped()
                p.header.seq = 1
                p.header = Header(stamp = rospy.Time.now(), frame_id = '/camera')
                                
                
                p.pose.position.x = float(absolute_x_position)
                p.pose.position.y = float(absolute_y_position)
                

                p.pose.orientation.x = float(yaw_camera)
                

                p.pose.orientation.w = 1.0


                # you could simultaneously display the data
                # on the terminal and to the log file
                rospy.loginfo(p)


                # publish the data to the topic using publish()
                pub.publish(p)

                rate.sleep(20)
            except:
                pass

        # ------------- Video Recording --------------
        if RECORD:
            if one_shot:  # Creates just one times VideoWriter Object
                result = cv2.VideoWriter("_TRACK.avi", cv2.VideoWriter_fourcc(*'XVID'), 25,
                                         (frame.shape[1], frame.shape[0]), isColor=True)
                one_shot = False
            result.write(frame)

        # --- Display the frame
        cv2.imshow('frame', frame)

        # --- use 'q' to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()

            # ------------- Plotting Purposes ----------------
            plt.title("Pose Estimation")
            plt.plot(x_position)
            plt.xlabel("detection_number")
            plt.ylabel("x position")
            plt.savefig("x_position.png")
            plt.close()

            plt.title("Pose Estimation")
            plt.plot(y_position)
            plt.xlabel("detection_number")
            plt.ylabel("y position")
            plt.savefig("y_position.png")
            plt.close()

            plt.title("Pose Estimation")
            plt.plot(yaw_angle)
            plt.xlabel("detection_number")
            plt.ylabel("yaw angle")
            plt.savefig("yaw_angle.png")
            plt.close()
            result.release()
            break


if __name__ == "__main__":
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
