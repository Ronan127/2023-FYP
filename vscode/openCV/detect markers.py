### Detect markers
import cv2
from cv2 import aruco
import numpy as np
import time

class MarkSearch :

    dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dict_aruco, parameters)
    

    def __init__(self, cameraID):
        self.cap = cv2.VideoCapture(cameraID)

    def get_markID(self):
        """
        Obtain marker id list from still image
        """
        ret, frame = self.cap.read()
        self.gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        corners, ids, rejectedImgPoints = self.detector.detectMarkers(self.gray)

        list_ids = np.ravel(ids)

        return list_ids

 
if __name__ == "__main__" :
    import cv2
    from cv2 import aruco
    import numpy as np
    import time

    dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    ### --- parameter --- ###
    cameraID = "./screen recording.mp4"
    cam0_mark_search = MarkSearch(cameraID)

    try:
        while cam0_mark_search.cap.isOpened():
            print(' ----- get_markID ----- ')
            print(cam0_mark_search.get_markID())
            cv2.imshow('frame', cam0_mark_search.gray)
    except KeyboardInterrupt:
        cam0_mark_search.cap.release()