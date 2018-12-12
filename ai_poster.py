import requests
import json


class AIPoster():
    # url = 'https://192.168.199.100:2444/api/ros_post'
    url = 'https://127.0.0.1:2444/api/ros_post'

    def post(self, json):
        r = requests.post(self.url, verify=False, json=json)
        print('http result:', r)
        return r

    def send_human_move(self, iccs_move):
        json = {'type': 'update_move', 'move': iccs_move}
        return self.post(json)

    def send_fen(self, fen):
        json = {'type': 'update_fen', 'fen': fen}
        return self.post(json)

    def send_init_board(self):
        json = {'type': 'init_board'}
        return self.post(json)

    def send_quest_move(self):
        json = {'type': 'ai_move'}
        return self.post(json).json()


if __name__ == '__main__':
    ac = AIPoster()
    import ipdb; ipdb.set_trace()