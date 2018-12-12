import cv2
import numpy as np

import rospy
from chess_detector import ChessDetector

cd = ChessDetector()
while(1):
    rospy.sleep(1)
    
cv2.destroyAllWindows()