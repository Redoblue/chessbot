# -*- coding: UTF-8 -*-

from __future__ import print_function

import numpy as np


class FenMap:

    FEN_LIST = ".rnbakcpRNBAKCP"
    INIT_FEN = "rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 0 1"
    
    def __init__(self):
        self.map = None
        self.from_fen(self.INIT_FEN)


    def __repr__(self):
        tmp_str = '\n'
        for i in range(9, -1, -1):
            for j in range(9):
                tmp_str += self.FEN_LIST[self.map[i, j]]
                tmp_str += ' '
            tmp_str += '\n'
        return tmp_str


    def __sub__(self, fen):
        tmp_fen = FenMap()
        tmp_fen.map = self.map - fen.map
        return tmp_fen


    def update_by_detection(self, locs, transform):
        self.map = np.zeros((10, 9), dtype=np.int8)
        for item in locs:
            x, y, w, h = map(int, item['coord'])
            rc = transform(x, y)
            rc = map(round, rc)
            rc = map(int, rc)
            self.map[rc[0], rc[1]] = item['label']


    @property
    def ones(self):
        ones_np = np.array(self.map > 0, dtype=np.int8)
        return ones_np


    def from_fen(self, fen):
        fens = fen[:-10].split('/')
        fens.reverse()
        assert len(fens) == 10
        # reset
        self.map = np.zeros((10, 9), dtype=np.int8)
        for i in range(len(fens)):
            j = 0 # 用来遍历map中的列
            for c in fens[i]:
                # 是数字的情况
                if c not in self.FEN_LIST:
                    j += int(c)
                else:
                    self.map[i, j] = self.FEN_LIST.index(c)
                    j += 1


    def to_fen(self, turn):
        assert self.map is not None
        fen = ''
        empty_count = 0
        for i in range(9, -1, -1):
            for j in range(9):
                if self.map[i, j] > 0:
                    if empty_count > 0:
                        fen += str(empty_count)
                        empty_count = 0
                    fen += self.FEN_LIST[self.map[i, j]]
                else:
                    empty_count += 1
            if empty_count > 0:
                fen += str(empty_count)
                empty_count = 0
            if i > 0:
                fen += '/'
        
        # 到谁了
        fen += ' b' if turn == 'ai' else ' w'
        fen += ' - -'
        fen += ' 0 1'

        return fen


    @staticmethod
    def count_fen(fen):
        count = 0
        for x in fen:
            if x in FenMap.FEN_LIST:
                count += 1
        print('count', count)
        return count


    @staticmethod
    def fmnp(ndarr):
        """
        format numpy ndarray
        """
        tmp_str = '\n'
        for i in range(9, -1, -1):
            for j in range(9):
                tmp_str += str(ndarr[i, j])
                tmp_str += ' '
            tmp_str += '\n'
        return tmp_str


if __name__ == '__main__':
    fen = Fen()
    fen.parse()
    import ipdb;ipdb.set_trace()
