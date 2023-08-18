# System information:
# - Linux Mint 18.1 Cinnamon 64-bit
# - Python 2.7 with OpenCV 3.2.0

import numpy
import cv2
from cv2 import aruco
import pickle
import glob


# ChAruco board variables
CHARUCOBOARD_ROWCOUNT = 6
CHARUCOBOARD_COLCOUNT = 4
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
MARKER_SIZE = 0.030
SQUARE_SIZE = 0.040

# Create constants to be passed into OpenCV and Aruco methods
CHARUCO_BOARD = aruco.CharucoBoard(
        size=(CHARUCOBOARD_COLCOUNT,CHARUCOBOARD_ROWCOUNT),
        squareLength=SQUARE_SIZE,
        markerLength=MARKER_SIZE,
        dictionary=ARUCO_DICT)

# Create the arrays and variables we'll use to store info like corners and IDs from images processed
corners_all = [] # Corners discovered in all images processed
ids_all = [] # Aruco ids corresponding to corners discovered
image_size = None # Determined at runtime

parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(ARUCO_DICT, parameters)
char_parameters = cv2.aruco.CharucoParameters()
char_detector = cv2.aruco.CharucoDetector(CHARUCO_BOARD)
counter =0


# This requires a set of images or a video taken with the camera you want to calibrate
# I'm using a set of images taken with the camera with the naming convention:
# 'camera-pic-of-charucoboard-<NUMBER>.jpg'
# All images used should be the same size, which if taken with the same camera shouldn't be a problem
cam = cv2.VideoCapture("./openCV/VID_20230817_125015.mp4")
while(cam.isOpened()):
    # Capturing each frame of our video stream
    ret, img = cam.read()
    if ret == True:
        if counter>0:
            counter -= 1
        else:
            # Grayscale the image
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # # Find aruco markers in the query image
            # corners, ids, _ = detector.detectMarkers(
            #         image=gray)

            # # Outline the aruco markers found in our query image
            # img = aruco.drawDetectedMarkers(
            #         image=img, 
            #         corners=corners)

            # Get charuco corners and ids from detected aruco markers
            charuco_corners, charuco_ids, marker_corners, marker_ids = char_detector.detectBoard(gray)

            # If a Charuco board was found, let's collect image/corner points
            # Requiring at least 5 square
            if charuco_ids is not None and len(charuco_ids) > 5:
                # Add these corners and ids to our calibration arrays
                corners_all.append(charuco_corners)
                ids_all.append(charuco_ids)
                print(len(ids_all))
                if not image_size:
                    image_size = gray.shape[::-1]
                
                #Wait x many frames before processing images again again
                counter = 10
                
                #Limit calibration to x many frames
                if len(ids_all)> 200:
                    break
        
            
        #     # Draw the Charuco board we've detected to show our calibrator the board was properly detected
        #     img = aruco.drawDetectedCornersCharuco(
        #             image=img,
        #             charucoCorners=charuco_corners,
        #             charucoIds=charuco_ids)
        
        #     # If our image size is unknown, set it now

        #     # Reproportion the image, maxing width or height at 1000

        # # Pause to display each image, waiting for key press
        # proportion = max(img.shape) / 1000.0
        # img = cv2.resize(img, (int(img.shape[1]/proportion), int(img.shape[0]/proportion)))
        # cv2.imshow('Charuco board', img)
        # cv2.waitKey(1)
    # Exit at the end of the video on the 'q' keypress
    if ret == False:
        break
    

# Destroy any open CV windows
cv2.destroyAllWindows()
print("Finished looking through images")
# Make sure at least one image was found
if len(ids_all) < 1:
    # Calibration failed because there were no images, warn the user
    print("Calibration was unsuccessful. No images of charucoboards were found. Add images of charucoboards and use or alter the naming conventions used in this file.")
    # Exit for failure
    exit()

# Make sure we were able to calibrate on at least one charucoboard by checking
# if we ever determined the image size
if not image_size:
    # Calibration failed because we didn't see any charucoboards of the PatternSize used
    print("Calibration was unsuccessful. We couldn't detect charucoboards in any of the images supplied. Try changing the patternSize passed into Charucoboard_create(), or try different pictures of charucoboards.")
    # Exit for failure
    exit()

# Now that we've seen all of our images, perform the camera calibration
# based on the set of points we've discovered
calibration, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
        charucoCorners=corners_all,
        charucoIds=ids_all,
        board=CHARUCO_BOARD,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None)
    
# Print matrix and distortion coefficient to the console
print(cameraMatrix)
print(distCoeffs)
    
# Save values to be used where matrix+dist is required, for instance for posture estimation
# I save files in a pickle file, but you can use yaml or whatever works for you
f = open('calibration2.pckl', 'wb')
pickle.dump((cameraMatrix, distCoeffs, rvecs, tvecs), f)
f.close()
    
# Print to console our success
print('Calibration successful. Calibration file used: {}'.format('calibration.pckl'))

