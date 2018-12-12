# -*- coding: UTF-8 -*-

import numpy as np

class BotDetector:

    def __init__(self):
        self.pv_transformer = None
        self.board_depth_sample = None
        self.detected = False
        self.queue_count = 0
        self.stable_count = [0, 0]
        self.hand_exited = False
    
    # 初始化检测区域
    def init_detection_corners(self, corners):
        self.hand_corners = []
        self.hand_corners.append((corners[2][0]-200, corners[2][1]-100))
        self.hand_corners.append((corners[3][0]+200, corners[3][1]-100))
        self.hand_corners.append((corners[2][0]-200, corners[2][1]))
        self.hand_corners.append((corners[3][0]+200, corners[3][1]))


    def set_transfomer(self, transformer):
        self.pv_transformer = transformer


    def reset(self):
        self.detected = False
        self.hand_exited = False


    def detect_hand(self, depth):
        if not self.pv_transformer.is_available():
            self.detected = False
            return
        tmp_depth_sample = []
        hand_corners_np = np.array(self.hand_corners)
        vecu = (hand_corners_np[1] - hand_corners_np[0])/40.0
        vecv = (hand_corners_np[2] - hand_corners_np[0])/20.0
        for i in range(40):
            for j in range(10):
                z = hand_corners_np[0] + i * vecu + j * vecv
                uv = z.tolist()
                u, v = map(int, z)
                d = depth[v, u]
                tmp_depth_sample.append(d)
        tmp_depth_sample = np.array(tmp_depth_sample)

        # 初始化棋盘深度向量
        if self.board_depth_sample is None:
            self.board_depth_sample = tmp_depth_sample
            print("hand detector\t initialized.")
            return

        # 判断是否有robot arm 
        self.stable_count[0] += 1
        mean_diff = abs(self.board_depth_sample.mean() - tmp_depth_sample.mean())

        # print(self.diff_queue.max())
        if mean_diff < 10.0:
            self.detected = False
        else:
            self.detected = True


    def is_available(self):
        return self.pv_transformer is not None


    def is_hand_detected(self):
        return self.detected
