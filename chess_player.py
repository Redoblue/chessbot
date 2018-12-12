#!/usr/bin/python
# -*- coding: UTF-8 -*-

import os
import numpy as np
from copy import deepcopy
import re
import rospy

from bot_mover import BotMover
from fenmap import FenMap
import utils
from utils import kill

from chessbot_control.msg import CornerPoint, BotReady

# joint states
IDLE_JOINT = [2.336195230484009, -2.2609503904925745, 2.2053871154785156, -1.5014928022967737, -1.5506561438189905, 0.0384644940495491]
IDLE_POSE = [0.23280793, -0.07017497,  0.16978045, -1.11387969,  2.91353807, 0.03724892]


class ChessPlayer:

    def __init__(self):
        self.mover = BotMover()
        self.fen = FenMap()
        self.yaml = "config.yaml"
        # self.tip_qua = {"h": [0.707,    0,      0.707,  0],
        #                 "i": [0.669,    0.230,  0.669,  -0.230],
        #                 "v": [0.533,    0.465,  0.532,  -0.465]}
        self.tip_qua = {
            "h": [-1.29792579,  2.84883972, -0.00688743],
            "i": [-2.28861281,  2.13473703, 0.000318388],
            "v": [-2.93226859,  1.09678566, 0.0072153]}
        self.corners = {}
        self.tmp_corners = {}
        self.hand_refined = False

        # init params
        self.init_params()

        # add collision
        # self.controller.add_collision(self.corners.values())

        # go to initial state
        raw_input("============ Press ENTER to go to the initial state")
        self.reset()
        self.mover.release()


    def init_params(self):
        try:
            corners = utils.read_yaml('corner_config_xyz.yaml')
            for i, c in enumerate(corners):
                self.corners[i] = c
        except:
            pass


    def handle_corner_point(self, msg):
        assert msg is not None
        xyz = [msg.point.x, msg.point.y, msg.point.z]
        self.tmp_corners[msg.which] = xyz

        if len(self.tmp_corners) == 4 and not self.hand_refined:
            # refine = raw_input("Refine corners (Y/n): ")
            refine = 'n'
            if refine == 'Y' or refine == 'y':
                self.refine_corners(self.tmp_corners)
                self.reset()
            self.hand_refined = True

            # tell detector that the bot is ready
            br = BotReady()
            self.pub_bot_ready.publish(br)
            print('bot ready published')


    def refine_corners(self, tmp_corners):
        """
        go to before refinement
        """
        real_corners = []
        for i in range(4):
            _ = raw_input("Press ENTER to refine corner {}".format(i))
            pose = tmp_corners[i]
            if len(pose) == 3:
                pose[2] += 0.025
                pose.extend(self.tip_qua['h'])
                self.mover.move_to_pose_state(pose)
                self.mover.grasp()

                _ = raw_input("Press ENTER to sample pose {}".format(i))
                pose_state = self.mover.pose_state
                real_corners.append(pose_state[:3])
                self.corners[i] = pose_state[:3]
        utils.save_yaml('corner_config_xyz.yaml', real_corners)
        print("Corners saved in yaml.")


    def get_grasp_angle(self, r, c):
        """
        根据当前的位置，返回对该位置抓取的角度
        """
        k = 'h'
        if c-1<0 and not self.fen.map[r, c+1]:
            k = 'h'
        elif c+1>8 and not self.fen.map[r, c-1]:
            k = 'h'
        elif not self.fen.map[r, c-1] and not self.fen.map[r, c+1]:
            k = 'h'
        elif r-1<0 and not self.fen.map[r+1, c]:
            k = 'v'
        elif r+1>9 and not self.fen.map[r-1, c]:
            k = 'v'
        elif not self.fen.map[r-1, c] and not self.fen.map[r+1, c]:
            k = 'v'
        else:
            # istatic
            k = 'i'
        return k


    def interp_rc(self, r, c):
        coord = utils.interp2d(self.corners.values(), r, c)
        r, c = utils.round_int(r), utils.round_int(c)

        # get tip angle

        k = self.get_grasp_angle(r, c)
        coord.extend(self.tip_qua[k])
        return coord


    def goto_rc(self, r, c):
        # 走之前先升起来
        if not self.is_idle():
            pv_via = self.mover.pose_vector
            pf_via_over = self.get_pose_over(pv_via)
            self.mover.move_pose(pf_via_over)

        # 计算目的点和上位点
        pv_to = self.interp_rc(r, c)
        pv_to_over = self.get_pose_over(pv_to)

        # 行走
        self.mover.move_pose(pv_to_over)
        self.mover.move_pose(pv_to)

    def get_pose_over(self, pose):
        pose_over = deepcopy(pose)
        pose_over[2] += 0.04
        return pose_over

    def reset(self):
        if not self.is_idle():
            pv = self.mover.pose_vector
            pvo = self.get_pose_over(pv)
            self.mover.move_pose(pvo)
            self.mover.move_joint(IDLE_JOINT)

    def is_idle(self):
        diff = np.array(self.mover.joint_vector) - np.array(IDLE_JOINT)
        # print(abs(diff).sum())
        return abs(diff).sum() < 0.1


    @staticmethod
    def parse_code(code):
        # code pattern: 2,3-4,6
        c2i = {x:i for i,x in enumerate('abcdefghi')}
        assert re.match("[a-i][0-9][a-i][0-9]$", code) is not None
        c1 = c2i[code[0]]
        r1 = int(code[1])
        c2 = c2i[code[2]]
        r2 = int(code[3])

        return (r1, c1, r2, c2)

    def move(self, pr1, pc1, pr2, pc2):
        r1 = utils.round_int(pr1)
        c1 = utils.round_int(pc1)
        r2 = utils.round_int(pr2)
        c2 = utils.round_int(pc2)
        print(r1, c1, r2, c2)
        assert 0 <= r1 <= 9 and 0 <= c1 <= 8 and 0 <= r2 <= 9 and 0 <= c2 <= 8
        if not self.fen.map[r1, c1]:
            rospy.loginfo("[WARN]\tNo piece found")
            return

        if self.fen.map[r2, c2]:
            self.remove(pr2, pc2)

        print("grab pos:", pr1, pc1)
        # import ipdb; ipdb.set_trace()
        self.goto_rc(pr1, pc1)
        self.mover.grasp()
        self.goto_rc(r2, c2)
        self.mover.release()
        self.reset()

    def remove(self, r, c):
        self.goto_rc(r, c)
        self.mover.grasp()
        self.reset()
        self.mover.release()


def main():
    try:
        rospy.init_node('chess_player', anonymous=True)
        player = ChessPlayer()

        import ipdb; ipdb.set_trace()

        rospy.spin()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
