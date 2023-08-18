import numpy as np
import cv2 as cv
import sys

dict_aruco = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
parameters = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dict_aruco, parameters)

img = cv.imread("./screen test wide 2.jpeg")
if img is None:
 sys.exit("Could not read the image.")
corners, ids, rejectedImgPoints = detector.detectMarkers(img)
cv.aruco.drawDetectedMarkers(img, corners, ids)
cv.imshow('frame', img)
sk = cv.waitKey(0)




# while cap.isOpened():
#  ret, frame = cap.read()
#  # if frame is read correctly ret is True
#  if not ret:
#     print("Can't receive frame (stream end?). Exiting ...")
#     break

#  gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
#  corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
#  cv.aruco.drawDetectedMarkers(gray, corners, ids)
#  cv.imshow('frame', gray)
 
 
#  print(ids)
#  if cv.waitKey(1) == ord('q'):
#     break
# cap.release()
# cv.destroyAllWindows()