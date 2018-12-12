# import the necessary packages
import numpy as np
import cv2

import utils


class PerspectiveViewTransformer:

	def __init__(self):
		self.rc2xy = None
		self.xy2rc = None

	def set_transformer(self, pts, nrow=10, ncol=9):
		if not isinstance(pts, np.ndarray):
			pts = np.array(pts, dtype=np.float32)

		# obtain a consistent order of the points and unpack them
		# individually
		dst = utils.order_points(pts)
		dst = np.array(dst, dtype=np.float32)
		(tl, tr, br, bl) = dst

		# compute the width of the new image, which will be the
		# maximum distance between bottom-right and bottom-left
		# x-coordiates or the top-right and top-left x-coordinates
		# widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
		# widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
		widthA = np.linalg.norm(br - bl)
		widthB = np.linalg.norm(tr - tl)
		maxWidth = max(int(widthA), int(widthB))

		# compute the height of the new image, which will be the
		# maximum distance between the top-right and bottom-right
		# y-coordinates or the top-left and bottom-left y-coordinates
		# heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
		# heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
		heightA = np.linalg.norm(tr - br)
		heightB = np.linalg.norm(tl - bl)
		maxHeight = max(int(heightA), int(heightB))

		# now that we have the dimensions of the new image, construct
		# the set of destination points to obtain a "birds eye view",
		# (i.e. top-down view) of the image, again specifying points
		# in the top-left, top-right, bottom-right, and bottom-left
		# order
		src = np.array([
			[nrow-1, 0],
			[nrow-1, ncol-1],
			[0, ncol-1],
			[0, 0]], dtype=np.float32)

		# compute the perspective transform matrix and then apply it
		self.rc2xy = cv2.getPerspectiveTransform(src, dst)
		self.xy2rc = cv2.getPerspectiveTransform(dst, src)

	def transform_rc2xy(self, row, col):
		if self.is_available():
			rc = np.array([[row, col]], dtype=np.float32).reshape(-1, 1, 2)
			xy = cv2.perspectiveTransform(rc, self.rc2xy)
			return xy.reshape(2).tolist()
		else:
			return None

	def transform_xy2rc(self, x, y):
		if self.is_available():
			xy = np.array([[x, y]], dtype=np.float32).reshape(-1, 1, 2)
			rc = cv2.perspectiveTransform(xy, self.xy2rc)
			return rc.reshape(2).tolist()
		else:
			return None

	def is_available(self):
		return self.rc2xy is not None and self.xy2rc is not None

