# -*- coding: UTF-8 -*-

from node_chess_player import ChessPlayer
from ai_poster import AIPoster

import rospy
import smach
import smach_ros


def main():

    rospy.init_node('chessbot_main')

    ac = AIPoster()
    cp = ChessPlayer()

    state = 'WAIT_HUMAN'
    rospy.init_node('chessbot_main')
    rospy.loginfo("Initializing game")
    ac.send_init_board()
    cp.show_fen()

    strategy = ['b2e2', 'h0g2', 'c3c4', 'b0c2', 'c2a1']
    count = 0
    while(True):
        # move = raw_input("Human move: ")
        move = strategy[count]
        raw_input("Press to go {}".format(move))
        # move = strategy[count]
        count += 1
        print('[HUMAN PLAYER]: {}'.format(move))
        ac.send_human_move(move)
        xyxy = cp.parse_code(move)

        cp.fake_move(*xyxy)
        cp.show_fen()


        # rospy.sleep(2)
        json = ac.send_quest_move()
        fen = json[u'fen']
        move = json[u'result']
        xyxy = cp.parse_code(move)
        print('[ROBOT PLAYER]: {}'.format(move))
        cp.move(*xyxy)

        cp.parse_fen(fen)
        cp.show_fen()


if __name__ == '__main__':
    main()