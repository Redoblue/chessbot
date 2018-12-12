import cv2
import numpy as np


class FlannSift:

    FLANN_INDEX_KDTREE = 0

    def __init__(self):
        self.sift = cv2.xfeatures2d.SIFT_create()

        # FLANN parameters
        index_params = dict(algorithm = self.FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50)   # or pass empty dictionary
        self.flann = cv2.FlannBasedMatcher(index_params,search_params)

    def extract(self, img):
        _, des = self.sift.detectAndCompute(img, None)
        return des

    def match(self, des1, des2):
        matches = self.flann.knnMatch(des1, des2, k=2)
        #import ipdb; ipdb.set_trace()

        # ratio test as per Lowe's paper
        num_good = 0
        for i,(m,n) in enumerate(matches):
            if m.distance < 0.7*n.distance:
                num_good += 1
        return num_good