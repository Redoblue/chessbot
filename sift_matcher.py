import cv2
import time
from copy import deepcopy
import numpy as np
from collections import defaultdict

from flann_sift import FlannSift


class SiftMatcher:

    def __init__(self):
        self.sift_features = None
        self.sift = FlannSift()

    def update_features(self, sift_f_dict):
        self.sift_features = sift_f_dict

    def extract_features(self, img, locs, pv_transformer):
        # cloned_color = deepcopy(img)
        # for item in locs:
        #     xy = item['coord'][:2]
        #     xy = map(int, xy)
        #     cv2.circle(cloned_color, tuple(xy), 3, (0, 0, 255), -1)
        # name = "sift-{}".format(time.time())
        # # cv2.namedWindow(name)
        # # cv2.imshow(name, cloned_color)
        # # cv2.waitKey(10000)
        # cv2.imwrite('sift/{}.png'.format(name), cloned_color)
        # import ipdb; ipdb.set_trace()
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # img = cv2.equalizeHist(img)
        # img = np.array(((img-img.min())/float(img.max()-img.min()))*255, dtype=np.uint8)
        # print(img)
        feature_dict = defaultdict(lambda: None)
        for loc in locs:
            xywh = loc['coord']
            crop = self.crop_image(img, xywh)
            des = self.sift.extract(crop)

            rc = pv_transformer.transform_xy2rc(*xywh[:2])
            rc = map(round, rc)
            r, c = map(int, rc)

            feature_dict[r*9+c] = des
        return feature_dict

    def compare(self, f_dict):
        min_pos, min_value = [], 1000
        for i in range(10):
            for j in range(9):
                f1 = f_dict[i*9+j]
                f2 = self.sift_features[i*9+j]

                if f1 is not None and f2 is not None:
                    nm = self.sift.match(f1, f2)
                    if nm < min_value:
                        min_value = nm
                        min_pos = (i, j)
                    # min_change.append((i, j, nm))
                    # if nm < 15:
                    #     change_pos.append((i, j))
                # elif f1 is None:
                #     change_pos.append((i, j))
        return min_pos

    def crop_image(self, img, xywh):
        x, y, w, h = map(int, xywh)
        anchor_x = x - int(w / 2.0)
        anchor_y = y - int(h / 2.0)

        crop = img[anchor_y: anchor_y + h, anchor_x: anchor_x + w]
        crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)        
        return crop

    def is_available(self):
        return self.sift_features is not None

    