#!/usr/bin/env python
#source activate py3 -- run code in virtualEnv
#source deactivate 
 
import cv2
import numpy as np
import math
 
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def draw_axis(img, R, t, K):
	# unit is mm
	rotV, _= cv2.Rodrigues(R)
	points = np.float32([[100, 0, 0], [0, 100, 0], [0, 0, 100], [0, 0, 0]]).reshape(-1, 3)
	axisPoints, _ = cv2.projectPoints(points, rotV, t, K, (0, 0, 0, 0))
	cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[0].ravel()), (255,0,0), 3)
	cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[1].ravel()), (0,255,0), 3)
	cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[2].ravel()), (0,0,255), 3)
	return img
 
# Calculates rotation matrix to euler angles
#Returns in radians
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
 
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])



# Read Image
im = cv2.imread("headPose.jpg");
size = im.shape
     
#2D image points. If you change the image, you need to change vector


#take in the points from the csv files
#possibly just these small ones at first
image_points = np.array([
                            (359, 391),     # Nose tip
                            (399, 561),     # Chin
                            (337, 297),     # Left eye left corner
                            (513, 301),     # Right eye right corne
                            (345, 465),     # Left Mouth corner
                            (453, 469)      # Right mouth corner
                        ], dtype="double")
 
# 3D model points.
#points that are calculated in generally assumed transformation matrix
model_points = np.array([
                            (0.0, 0.0, 0.0),             # Nose tip
                            (0.0, -330.0, -65.0),        # Chin
                            (-225.0, 170.0, -135.0),     # Left eye left corner
                            (225.0, 170.0, -135.0),      # Right eye right corne
                            (-150.0, -150.0, -125.0),    # Left Mouth corner
                            (150.0, -150.0, -125.0)      # Right mouth corner
                         
                        ])
 
 
# Camera internals - calculated
# things that are not fixed and can change based on the image
#do not depend on camera specifics
focal_length = size[1]
center = (size[1]/2, size[0]/2)
camera_matrix = np.array(
                         [[focal_length, 0, center[0]],
                         [0, focal_length, center[1]],
                         [0, 0, 1]], dtype = "double"
                         )
 
print ("Camera Matrix :\n {0}".format(camera_matrix))
 
dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
(success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs)
 
print ("Rotation Vector:\n {0}".format(rotation_vector))
print ("Translation Vector:\n {0}".format(translation_vector))
 
 
# Project a 3D point (0, 0, 1000.0) onto the image plane.
# We use this to draw a line sticking out of the nose
 
 
(nose_end_point2D, jacobian) = cv2.projectPoints(np.array([(0.0, 0.0, 1000.0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)
 
########## ADDING CALCULATION FOR EULER ANGLES #############
rotation_matrix = np.zeros(shape=(3,3))
cv2.Rodrigues(rotation_vector, rotation_matrix)
print ("rotation matrix")
print (rotation_matrix)

roll_pitch_yaw = rotationMatrixToEulerAngles(rotation_matrix)

print ("roll")
print (roll_pitch_yaw[0])
print ("pitch")
print (roll_pitch_yaw[1])
print ("yaw")
print (roll_pitch_yaw[2])

for p in image_points:
    cv2.circle(im, (int(p[0]), int(p[1])), 3, (0,0,255), -1)
 
# p1 = ( int(image_points[0][0]), int(image_points[0][1]))
# p2 = ( int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))
 
# cv2.line(im, p1, p2, (255,0,0), 2)

draw_axis(im, rotation_vector, translation_vector, camera_matrix)
 
# Display image
cv2.imshow("Output", im)
cv2.waitKey(0)