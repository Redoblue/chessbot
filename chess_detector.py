# -*- coding: UTF-8 -*-

import time
import cv2
import numpy as np
import thread
from copy import deepcopy

# import detection module
import sys
sys.path.append('/home/baoliqiang/Workspace/chessbot/src/chessbot_yolo')
from run import detect as yolo_detect

sys.path.append('/home/baoliqiang/Workspace/chessbot/src/chessbot_resnet')
from run_mobilenet_classify import classify as mobile_classify

import utils

class ChessDetector:

    def __init__(self):
        pass

    # @utils.timer
    # def detect(self, img, corners, num=None):
    #     _, detect_res = yolo_detect(img, corners)

    #     if num is not None:
    #         if isinstance(num, int):
    #             while len(detect_res) != num:
    #                 print('detected {}/{} chesses, redetecting...'.format(len(detect_res), num))
    #                 _, detect_res = yolo_detect(img, corners)
    #         elif isinstance(num, tuple):
    #             while len(detect_res) not in num:
    #                 print('detected {}/{}-{} chesses, redetecting...'.format(len(detect_res), num[0], num[1]))
    #                 _, detect_res = yolo_detect(img, corners)

    #     # classify now
    #     for res in detect_res:
    #         xywh = res['coord']
    #         crop = self.crop_image(img, xywh)
    #         res['label'] = classify(crop)

    #     return detect_res

    @staticmethod
    def detect(img, corners):
        _, detect_res = yolo_detect(img, corners)
        return detect_res

    @staticmethod
    def classify(img):
        return mobile_classify(img)

    @staticmethod
    def crop_image(img, xywh):
        x, y, w, h = map(int, xywh)
        anchor_x = x - int(w / 2.0)
        anchor_y = y - int(h / 2.0)
        crop = img[anchor_y: anchor_y + h, anchor_x: anchor_x + w]
        crop = cv2.imencode('.jpg', crop)[1].tostring()
        return crop


    @staticmethod
    def show(img, detect_res, atonce=True):
        cloned_img = deepcopy(img)
        for item in detect_res:
            xywh = item['coord']
            x, y, w, h = map(int, xywh)
            anchor_x = x - int(w / 2.0)
            anchor_y = y - int(h / 2.0)

            corners = []
            corners.append((anchor_x, anchor_y))
            corners.append((anchor_x, anchor_y+h))
            corners.append((anchor_x+w, anchor_y+h))
            corners.append((anchor_x+w, anchor_y))
            corners_np = np.array(corners)
            cv2.circle(cloned_img, (x, y), 4, (0, 0, 255), -1)
            cv2.polylines(cloned_img, [corners_np], True, (0, 255, 0), 3)
            cv2.putText(cloned_img, str(item['label']), (x-10, y+30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)

        if atonce:
            #画图
            cv2.imshow('data', cloned_img)
            cv2.waitKey(5000)
        else:
            return cloned_img

    
if __name__ == '__main__':
    cd = ChessDetector()
    img = '/home/baoliqiang/Workspace/chessbot/src/chessbot_control/scripts-data/data/2018-11-20-15:41:27/18.png'
    img = cv2.imread(img)
    corners = [[400, 885], [1200, 884], [1200, 200], [400, 200]]
    detect_res = cd.detect(img, corners)
    cd.show(img, detect_res)
