# The following code is used to watch a video stream, detect Aruco markers, and use
# a set of markers to determine the posture of the camera in relation to the plane
# of markers.
#
# Assumes that all markers are on the same plane, for example on the same piece of paper
#
# Requires camera calibration (see the rest of the project for example calibration)

import numpy as np
import cv2
import cv2.aruco as aruco
import os
import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Check for camera calibration data
if not os.path.exists('./calibration.pckl'):
    print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
    exit()
else:
    f = open('calibration.pckl', 'rb')
    (cameraMatrix, distCoeffs, _, _) = pickle.load(f)
    f.close()
    if cameraMatrix is None or distCoeffs is None:
        print("Calibration issue. Remove ./calibration.pckl and recalibrate your camera with CalibrateCamera.py.")
        exit()

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters()
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMETERS)
MARKER_SIZE = 0.030

# Create grid board object we're using in our stream
# board = aruco.GridBoard_create(
#         markersX=2,
#         markersY=2,
#         markerLength=0.09,
#         markerSeparation=0.01,
#         dictionary=ARUCO_DICT)

# Create vectors we'll be using for rotations and translations for postures
rvecs, tvecs = [0]*50, [0]*50

def process_image(QueryImg):

    gray = cv2.cvtColor(QueryImg, cv2.COLOR_BGR2GRAY)
    # Detect Aruco markers
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

    # Refine detected markers
    # Eliminates markers not part of our board, adds missing markers to the board
    # corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
    #         image = gray,
    #         board = board,
    #         detectedCorners = corners,
    #         detectedIds = ids,
    #         rejectedCorners = rejectedImgPoints,
    #         cameraMatrix = cameraMatrix,
    #         distCoeffs = distCoeffs)   

    ###########################################################################
    # TODO: Add validation here to reject IDs/corners not part of a gridboard #
    ###########################################################################

    # Outline all of the markers detected in our image
    QueryImg = aruco.drawDetectedMarkers(QueryImg, corners, ids)

    if ids is not None and len(ids) > 0:
        # Estimate the posture of the gridboard, which is a construction of 3D space based on the 2D video 
        #pose, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs)
        #if pose:
        #    # Draw the camera posture calculated from the gridboard
        #    QueryImg = aruco.drawAxis(QueryImg, cameraMatrix, distCoeffs, rvec, tvec, 0.3)
        # Estimate the posture per each Aruco marker
        for i in range(0, len(ids)):
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE, cameraMatrix, distCoeffs)           
            QueryImg = cv2.drawFrameAxes(QueryImg, cameraMatrix, distCoeffs, rvec, tvec, MARKER_SIZE)

            rvecs[ids[i][0]]=rvec[0][0]
            tvecs[ids[i][0]]=tvec[0][0]
        
        # Reproportion the image, maxing width or height at 1000
        proportion = max(QueryImg.shape) / 1000.0
        QueryImg = cv2.resize(QueryImg, (int(QueryImg.shape[1]/proportion), int(QueryImg.shape[0]/proportion)))
        # Display our image
        cv2.imshow('QueryImage', QueryImg) 
    return QueryImg
    
def process_video(VideoStream):
    cam = cv2.VideoCapture(VideoStream)
    while(cam.isOpened()):
        # Capturing each frame of our video stream
        ret, QueryImg = cam.read()
        if ret == True:
            process_image(QueryImg)

        # Exit at the end of the video on the 'q' keypress
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# process_video('./openCV/20230816_161339.mp4')
img = cv2.imread("./openCV/Calibration4/IMG_20230817_100610.jpg")
img2=process_image(img)

cv2.waitKey(0)
img_undistort =cv2.undistort(img2,cameraMatrix,distCoeffs,None)
cv2.imshow('Undistorted', img_undistort) 
cv2.waitKey(0)
# cv2.destroyAllWindows()



# Calculate best fit plane

N_POINTS = 12
TARGET_X_SLOPE = 2
TARGET_y_SLOPE = 3
TARGET_OFFSET  = 5
EXTENTS = 5
NOISE = 5

def FitPlaneVisualise(tvecs):
    xs = []
    ys = []
    zs = []
    for i in range(N_POINTS):
        xs.append(tvecs[i][0])
        ys.append(tvecs[i][1])
        zs.append(tvecs[i][2])


    # plot raw data
    plt.figure()
    ax = plt.subplot(111, projection='3d')
    ax.scatter(xs, ys, zs, color='b')
    ax.set_box_aspect([ub - lb for lb, ub in (getattr(ax, f'get_{a}lim')() for a in 'xyz')])

    # do fit
    tmp_A = []
    tmp_b = []
    for i in range(len(xs)):
        tmp_A.append([xs[i], ys[i], 1])
        tmp_b.append(zs[i])
    b = np.matrix(tmp_b).T
    A = np.matrix(tmp_A)
    fit = (A.T * A).I * A.T * b
    errors = b - A * fit
    residual = np.linalg.norm(errors)

    print("solution: %f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
    print("errors:")
    print(errors)
    print("residual: {}".format(residual))

    # plot plane
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
                    np.arange(ylim[0], ylim[1]))
    Z = np.zeros(X.shape)
    for r in range(X.shape[0]):
        for c in range(X.shape[1]):
            Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
    ax.plot_wireframe(X,Y,Z, color='k')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()
    return fit

fit = FitPlaneVisualise(tvecs)

theta=np.arctan(-fit[1,0])
Rx=np.matrix([[1, 0, 0],
              [0,np.cos(theta),-np.sin(theta)],
              [0,np.sin(theta),np.cos(theta)]])

theta=np.arctan(fit[0,0])
Ry=np.matrix([[np.cos(theta), 0, np.sin(theta)],
              [0,1,0],
              [-np.sin(theta),0,np.cos(theta)]])

tvecs2=[]
for i in range(0,N_POINTS):
    tvecs2.append(np.ravel(np.matmul(Rx,tvecs[i])))
# fit2 = FitPlaneVisualise(tvecs2)


tvecs3=[]
for i in range(0,N_POINTS):
    tvecs3.append(np.ravel(np.matmul(Ry,tvecs2[i])))
fit3 = FitPlaneVisualise(tvecs3)

tvecs4 = []
for i in range(0,N_POINTS):
    tvecs4.append(tvecs3[i]-[0,0,np.ravel(fit3[2])[0]])
# fit4 = FitPlaneVisualise(tvecs4) 

tvecs5 = []
for i in range(0,N_POINTS):
    tvecs5.append([tvecs4[i][0]-tvecs4[1][0],tvecs4[i][1]-tvecs4[1][1], 0])
fit5 = FitPlaneVisualise(tvecs5) 


theta=np.arctan((tvecs5[5][0]/tvecs5[5][1]+tvecs5[9][0]/tvecs5[9][1])/2)
Rz=np.matrix([[np.cos(theta),-np.sin(theta), 0],
              [np.sin(theta),np.cos(theta),0],
              [0,0,1]])

tvecs6=[]
for i in range(0,N_POINTS):
    tvecs6.append(np.ravel(np.matmul(Rz,tvecs5[i])))
fit6 = FitPlaneVisualise(tvecs6) 

pass
