#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sh
import time
import rospy
from std_msgs.msg import String
from chessbot_control.msg import ChessMove, Fen
from chessbot_control.srv import PreciseMove

# import params
from ai_poster import AIPoster
from node_chess_player import ChessPlayer
from speaker import Speaker


ai_poster = None
chess_player = None
speaker = Speaker()

pub_ai_fen = rospy.Publisher('/chessbot/fen', Fen, queue_size=4)

# PreciseMove service
# rospy.wait_for_service('chessbot/precise_move')
# get_precise_move = rospy.ServiceProxy('chessbot/precise_move', PreciseMove)


def request_ai_move(human_move):
    global chess_player, ai_poster, pub_ai_fen, speaker
    json = ai_poster.send_quest_move()
    fen = json[u'fen']
    move = json[u'result']
    xyxy = ChessPlayer.parse_code(move)
    print('[ROBOT PLAYER]: {}'.format(move))
    
    # 更新human走后的地图
    human_xyxy = ChessPlayer.parse_code(human_move)
    chess_player.fen.map[human_xyxy[0], human_xyxy[1]] = 0
    chess_player.fen.map[human_xyxy[2], human_xyxy[3]] = 1

    # 我们在这里使用PreciseMove查询要下的位置的棋子的确切位置
    # import ipdb; ipdb.set_trace()
    # pxyxy = get_precise_move(*xyxy)
    # pxyxy = [pxyxy.fr1, pxyxy.fc1, pxyxy.fr2, pxyxy.fc2]
    # print('precise move', pxyxy)

    # 机器人走步
    speaker.say('my_turn')
    rospy.sleep(1)
    chess_player.move(*xyxy)

    # 更新机器人走后的地图
    chess_player.fen.update_by_fen(fen)
    # chess_player.fen.show()

    # 发布ai传送的局面
    fen_msg = Fen()
    fen_msg.fen = fen
    pub_ai_fen.publish(fen_msg)

def on_chess_move(data):
    global ai_poster
    
    hmr = ai_poster.send_human_move(data.move)
    print('response', hmr)
    rospy.loginfo("I heard %s",data.move)

    # ask ai to move
    request_ai_move(data.move)


def main():
    rospy.init_node('chessbot_main')

    global ai_poster, chess_player
    ai_poster = AIPoster()
    ai_poster.send_init_board()

    chess_player = ChessPlayer()

    # subscribes
    rospy.Subscriber("/chessbot/chess_move", ChessMove, on_chess_move)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()