import cv2
import cv2.aruco as aruco

# Create ChArUco board, which is a set of Aruco markers in a chessboard setting
# meant for calibration
# the following call gets a ChArUco board of tiles 5 wide X 7 tall
gridboard = aruco.CharucoBoard(
        # squaresX=5, 
        # squaresY=7,
        size=(4,6),
        squareLength=0.040, 
        markerLength=0.030, 
        dictionary=aruco.getPredefinedDictionary(aruco.DICT_4X4_50))

# Create an image from the gridboard
img = gridboard.generateImage(outSize=(988, 1400))
cv2.imwrite("test_charuco_marrgined.jpg", img)

# Display the image to us
cv2.imshow('Gridboard', img)
# Exit on any key
cv2.waitKey(0)
cv2.destroyAllWindows()