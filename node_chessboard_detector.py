#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys
import os
import pyfiglet
import time
import numpy as np
import cv2
import sh
from copy import copy, deepcopy
from scipy.ndimage import filters
from cv_bridge import CvBridge, CvBridgeError
import thread, threading
import simplejson

import roslib
import rospy
import tf
import geometry_msgs
from sensor_msgs.msg import Image, PointCloud2
from chessbot_control.msg import *
from chessbot_control.srv import *

import utils
from utils import kill
# import params
from perspective_view_transformer import PerspectiveViewTransformer
from depth_plane_fitter import DepthPlaneFitter
from camera_coord_transformer import CameraCoordTransformer
from hand_detector import HandDetector
from bot_detector import BotDetector
from fenmap import FenMap
from speaker import Speaker
from logger import ChessLogger
from data_collector import DataCollector
from chess_detector import ChessDetector
from ai_poster import AIPoster
from chess_player import ChessPlayer


NROW = 10
NCOL = 9

logger = ChessLogger()


class ChessboardDetector:

    SHOW = True
    COLLECT_DATA = True

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('chessboard_detector', anonymous=True)

        # instances
        self.pv_transformer = PerspectiveViewTransformer()
        self.dp_fitter = DepthPlaneFitter()
        self.cc_transformer = CameraCoordTransformer()
        self.hand_detector = HandDetector()
        self.bot_detector = BotDetector()
        self.ai_poster = AIPoster()
        self.chess_player = ChessPlayer()
        self.speaker = Speaker()
        self.data_collector = DataCollector()
        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()

        # other variables
        self.marked = False
        self.chessboard_corners = []
        self.wait_for_human = True
        self.game_started = False
        self.in_main_loop = False
        self.semaphore = threading.Semaphore(1)

        # subscribed Topic
        self.sub_color = rospy.Subscriber(
            "/kinect2/hd/image_color_rect", Image, self.handle_color)
        self.sub_depth = rospy.Subscriber(
            "/kinect2/hd/image_depth_rect", Image, self.handle_depth)
        self.sub_points = rospy.Subscriber(
            "/kinect2/hd/points", PointCloud2, self.handle_points)


        try:
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down..."
            kill()
        # cv2.destroyAllWindows()


    def start_game(self):
        if hasattr(self, 'img_color') and self.pv_transformer.is_available():
            # 默认开局为32个棋子
            ini_count = int(raw_input('Please input initial number (32): ') or '32')
            detect_res = self.redetect(num=ini_count)
            # 初始局面更新
            self.chess_player.fen.update_by_detection(detect_res, self.pv_transformer.transform_xy2rc)
            # 初始化棋面
            self.ai_poster.send_init_board()
            self.ai_poster.send_fen(self.chess_player.fen.to_fen('human'))

            # logging
            logger.info('updated fen:')
            logger.info(self.chess_player.fen)

            # 在另一线程中采集数据
            if self.COLLECT_DATA:
                thread.start_new_thread(self.data_collector.collect, (self.img_color, detect_res, self.pv_transformer.transform_xy2rc))

            # 告诉用户开始下棋
            self.speaker.say('game_start', False)
            self.speaker.say('your_turn')


    def load_params(self):
        try:
            self.chessboard_corners = utils.read_yaml('corner_config_uv.yaml')
            return True
        except:
            return False


    def handle_color(self, msg):
        try:
            if self.hand_detector.is_available() and self.bot_detector.is_available():
                if not self.hand_detector.detected and not self.bot_detector.detected:
                    self.img_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            else:
                self.img_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)


    def handle_depth(self, msg):
        try:
            self.img_depth = self.bridge.imgmsg_to_cv2(msg)

            # detect hand
            if self.hand_detector.is_available():
                self.hand_detector.detect_hand(self.img_depth)
            if self.bot_detector.is_available():
                self.bot_detector.detect_hand(self.img_depth)
        except CvBridgeError as e:
            print(e)
        
        # initialize perspective view transformer and depth pland fitter
        if len(self.chessboard_corners) == 4:
            if not self.pv_transformer.is_available():
                self.pv_transformer.set_transformer(self.chessboard_corners)
            if not self.dp_fitter.is_available():
                self.dp_fitter.fit(self.img_depth, self.pv_transformer, show=False)
            if not self.hand_detector.is_available():
                self.hand_detector.set_transfomer(self.pv_transformer)
                self.hand_detector.init_detection_corners(self.chessboard_corners)
            if not self.bot_detector.is_available():
                self.bot_detector.set_transfomer(self.pv_transformer)
                self.bot_detector.init_detection_corners(self.chessboard_corners)

            if not self.game_started:
                self.start_game()
                self.game_started = True


    def handle_points(self, msg):
        if hasattr(self, 'img_color') and hasattr(self, 'img_depth'):
            # mark corners first
            if not self.marked:
                # load = raw_input("Load corners (Y/n): ")
                load = 'Y'
                if load == 'Y' or load == 'y':
                    self.load_params()
                    logger.info("corners loaded")
                else:
                    self.mark_corners_by_hand()
                    utils.save_yaml('corner_config_uv.yaml', self.chessboard_corners)
                self.marked = True
            else:
                if self.SHOW and hasattr(self, 'show_color'):
                    ########################################### display info ###########################################
                    # 画棋盘包围框
                    corners = copy(self.chessboard_corners)
                    xy3 = corners.pop()
                    corners.insert(2, xy3)
                    corners_np = np.array(corners)
                    cv2.polylines(self.show_color, [corners_np], True, (0, 255, 0))

                    # 画棋格
                    if self.pv_transformer.is_available():
                        for i in range(10):
                            for j in range(9):
                                z = self.pv_transformer.transform_rc2xy(i, j)
                                z = map(int, z)
                                cv2.circle(self.show_color, tuple(z), 2, (255, 0, 0), -1)
                    # 手检测区域
                    if self.hand_detector.is_available():
                        hand_corners = self.hand_detector.hand_corners
                        hand_corners_np = np.array(hand_corners)
                        vecu = (hand_corners_np[1] - hand_corners_np[0])/40.0
                        vecv = (hand_corners_np[2] - hand_corners_np[0])/20.0
                        for i in range(40):
                            for j in range(10):
                                z = hand_corners_np[0] + i * vecu + j * vecv
                                z = z.tolist()
                                z = map(int, z)
                                cv2.circle(self.show_color, tuple(z), 2, (255, 255, 0), -1)
                    # 机械臂检测区域
                    if self.bot_detector.is_available():
                        hand_corners = self.bot_detector.hand_corners
                        hand_corners_np = np.array(hand_corners)
                        vecu = (hand_corners_np[1] - hand_corners_np[0])/40.0
                        vecv = (hand_corners_np[2] - hand_corners_np[0])/20.0
                        for i in range(40):
                            for j in range(10):
                                z = hand_corners_np[0] + i * vecu + j * vecv
                                z = z.tolist()
                                z = map(int, z)
                                cv2.circle(self.show_color, tuple(z), 2, (255, 255, 0), -1)

                    resized_color = cv2.resize(self.show_color, (1280, 720))
                    cv2.imshow('image', resized_color)
                    cv2.waitKey(1)

                # 开启主逻辑
                thread.start_new_thread(self.main_loop, ())


    def main_loop(self):
        if self.wait_for_human and self.hand_detector.hand_exited and not self.bot_detector.detected:

            # logger.info('wait_for_human: {}, hand_exited: {}, bot_detected: {}'.format(self.wait_for_human, self.hand_detector.hand_exited, self.bot_detector.detected))

            # 检测当前棋面是否变化,加的约束为要么少一子，要么子不变
            detect_res = self.redetect(num=(self.chess_player.fen.ones.sum()-1, self.chess_player.fen.ones.sum()))
            tmp_fen = FenMap()
            tmp_fen.update_by_detection(detect_res, self.pv_transformer.transform_xy2rc)
            map_diff = tmp_fen.ones - self.chess_player.fen.ones

            # 棋面有变化
            if map_diff.any():
                # 互斥访问主循环
                self.semaphore.acquire()
                if self.in_main_loop:
                    print('already in main loop')
                    self.semaphore.release()
                    return
                self.in_main_loop = True
                self.semaphore.release()
                print('in main loop')

                # 暂时不等待人
                self.wait_for_human = False

                # logging
                logger.info("detected map change")
                logger.info('--------------------------------------------')
                logger.info('[current fen]:\n{}'.format(self.chess_player.fen))
                logger.info('[after  hand]:\n{}'.format(tmp_fen))
                logger.info('[diff       ]:\n{}'.format(FenMap.fmnp(map_diff)))

                # 在另一线程中采集数据
                if self.COLLECT_DATA:
                    thread.start_new_thread(self.data_collector.collect, (self.img_color, detect_res, self.pv_transformer.transform_xy2rc))

                # 检测认输机制
                # resign = False if (12 in tmp_fen.map[:3, 3:6]) else True
                # if resign:
                #     self.speaker.say('you_lose')
                #     time.sleep(3)
                #     kill()

                # 更新self.chess_player.fen
                self.chess_player.fen = tmp_fen
                # 语音-->思考中...
                self.speaker.say('thinking')
                time.sleep(1.5)
                # 发送human move
                self.ai_poster.send_fen(self.chess_player.fen.to_fen('ai'))

                ########################### 机械臂运动 ##############################
                try:
                    json = self.ai_poster.send_quest_move()
                    logger.info('json received: \n' + str(json))
                    fen = json[u'fen']
                    result = json[u'result']
                    move = json[u'move']

                    if result == u'you_win':
                        self.speaker.say('you_win')
                        time.sleep(3)
                        kill()
                    elif result == u'you_lose':
                        self.speaker.say('you_lose')
                        time.sleep(3)
                        kill()
                    elif result == u'resign':
                        self.speaker.say('you_win')
                        time.sleep(3)
                        kill()

                    print('[ROBOT PLAYER]: {}'.format(move))

                except simplejson.scanner.JSONDecodeError:
                    logger.info('system error!')

                xyxy = ChessPlayer.parse_code(move)
                logger.info('received fen: ' + fen)

                # 我们在这里使用PreciseMove查询要下的位置的棋子的确切位置
                # import ipdb; ipdb.set_trace()
                pxyxy = self.get_precise_move(xyxy)
                logger.info('coarse: ' + ','.join(map(str, xyxy)))
                logger.info('fine  : ' + ','.join(map(str, pxyxy)))

                # 在机械臂走之前我们要停止手的检测，防止机械臂长程吃出问题
                self.hand_detector.enabled = False

                # 机器人走步
                self.speaker.say('my_turn')
                rospy.sleep(1)
                self.chess_player.move(*pxyxy)

                # 将军的情况
                if result == u'checking':
                    self.speaker.say('check_mate')
                    time.sleep(2)

                # 等待机器人退出
                while self.bot_detector.detected:
                    time.sleep(0.2)

                ############################### 机械臂退出后 ####################################
                # 更新棋面
                self.chess_player.fen.from_fen(fen)
                # 等待玩家下棋
                self.wait_for_human = True
                # 开启手的检测
                self.hand_detector.enabled = True
                #　重置手检测
                self.hand_detector.reset()
                # 多线程提示
                self.speaker.say('your_turn')

                # logging
                logger.info('updated fen:')
                logger.info(self.chess_player.fen)
                logger.info("Waiting for HUMAN player...")

                # 退出主循环
                self.semaphore.acquire()
                self.in_main_loop = False
                self.semaphore.release()
            else:
                #　重置手检测
                self.hand_detector.reset()


    def redetect(self, num=None):
        detect_res = ChessDetector.detect(self.img_color, self.chessboard_corners)

        if num is not None:
            if isinstance(num, int):
                if len(detect_res) != num:
                    self.show_color = ChessDetector.show(self.img_color, detect_res, atonce=False)
                while len(detect_res) != num:
                    print('detected {}/{} chesses, redetecting...'.format(len(detect_res), num))
                    detect_res = ChessDetector.detect(self.img_color, self.chessboard_corners)
                    self.show_color = ChessDetector.show(self.img_color, detect_res, atonce=False)
            elif isinstance(num, tuple):
                if len(detect_res) not in num:
                    self.show_color = ChessDetector.show(self.img_color, detect_res, atonce=False)
                while len(detect_res) not in num:
                    print('detected {}/{}-{} chesses, redetecting...'.format(len(detect_res), num[0], num[1]))
                    detect_res = ChessDetector.detect(self.img_color, self.chessboard_corners)
                    self.show_color = ChessDetector.show(self.img_color, detect_res, atonce=False)

        # classify now
        for res in detect_res:
            xywh = res['coord']
            crop = ChessDetector.crop_image(self.img_color, xywh)
            res['label'] = ChessDetector.classify(crop)

        # 保存检测的结果
        self.board_info = detect_res

        # 添加了识别之后的棋盘
        self.show_color = ChessDetector.show(self.img_color, detect_res, atonce=False)

        return detect_res

    def get_precise_move(self, rcrc):
        """
        desc: 对于req中的r和c，我们返回在这个点的棋子的确切位置
        params: req (x, y)
        return: px, py
        """
        if not hasattr(self, 'board_info'):
            return rcrc
            
        r1, c1, r2, c2 = rcrc
        pr1, pc1, pr2, pc2 = rcrc
        for item in self.board_info:
            x, y, w, h = map(int, item['coord'])
            rc = self.pv_transformer.transform_xy2rc(x, y)
            rc_round = map(round, rc)
            rc_int = map(int, rc_round)
            if rc_int[0] == r1 and rc_int[1] == c1:
                pr1, pc1 = rc[0], rc[1]
            if rc_int[0] == r2 and rc_int[1] == c2:
                pr2, pc2 = rc[0], rc[1]
        return (pr1, pc1, pr2, pc2)


    def on_mouse_clicked(self, event, x, y, flags, param):
        """
        We manually mark the four corners
        """
        # if the left mouse button was clicked, record the starting
        # (x, y) coordinates and indicate that cropping is being performed
        if event == cv2.EVENT_LBUTTONDOWN and len(self.chessboard_corners) < 4:
            self.chessboard_corners.append((x, y))


    def mark_corners_by_hand(self):
        cv2.namedWindow("image")
        cv2.setMouseCallback("image", self.on_mouse_clicked)

        while len(self.chessboard_corners) < 4:
            cloned_color = self.img_color
            for xy in self.chessboard_corners:
                cv2.circle(cloned_color, xy, 3, (0,0,255), -1)
            cv2.imshow('image', cloned_color)
            cv2.waitKey(1)


if __name__ == '__main__':
    cb_detector = ChessboardDetector()

