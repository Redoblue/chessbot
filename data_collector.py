# -*- coding: UTF-8 -*-

import os
import numpy as np
import time
import cv2
import sh
import ast


class DataCollector:

    def __init__(self):
        self.save_path = 'data/{}'.format(time.strftime("%Y-%m-%d-%H:%M:%S", time.localtime()))
        self.count = 0

        # 创建文件夹
        self.make_dirs()

    def make_dirs(self):
        try:
            sh.mkdir('-p', self.save_path)
        except:
            pass


    def collect(self, img, detect_res, transform):
        # 图片路径
        img_path = '{}/{}.png'.format(self.save_path, self.count)

        # 保存图片
        cv2.imwrite(img_path, img)

        # 保存标签
        with open('{}/labels.txt'.format(self.save_path), 'a') as f:
            # 写入图片名
            f.write('{}.png ['.format(self.count))
            for item in detect_res:
                # 写入文件
                f.write(str(item) + ', ')

            # 终结
            f.write(']\n')

        print('SAVE {}'.format(self.count))

        # 更新编号
        self.count += 1

    @staticmethod
    def show_dir(dir):
        img_names = [item for item in os.listdir(dir) if 'png' in item]
        img_names.sort(key=lambda x: int(x[:-4]))
        num_imgs = len(img_names)
        if num_imgs == 0:
            print('No image found!')
            return
        
        # 先读标签
        with open('{}/labels.txt'.format(dir)) as f:
            data = f.readlines()

        # 创建显示窗口
        cv2.namedWindow('data')

        count = 0
        while True:
            cur_img = img_names[count]
            cur_img_path = '{}/{}'.format(dir, cur_img)
            
            # 读取图片
            img = cv2.imread(cur_img_path)

            # 读取标签
            img_index = int(cur_img[:-4])
            label_str = data[img_index]
            label_json_str = label_str[label_str.index('['):-1]
            label_list = ast.literal_eval(label_json_str)

            # 画出标签
            cv2.putText(img, cur_img[:-4], (20, 60), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 255, 255), 4)
            for item in label_list:
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
                # cv2.circle(img, (x, y), 3, (0, 0, 255), -1)
                cv2.polylines(img, [corners_np], True, (0, 255, 0), 3)
                cv2.putText(img, str(item['label']), (x-10, y+30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)

            #画图
            cv2.imshow('data', img)

            key = cv2.waitKey(1)
            if key == 83:
                count = min(count+1, num_imgs-1)
            elif key == 81:
                count = max(count-1, 0)
            elif key == ord('q'):
                cv2.destroyAllWindows()
                break


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--dir', default=None, type=str, help='dir to scan')
    args = parser.parse_args()

    DataCollector.show_dir(args.dir)