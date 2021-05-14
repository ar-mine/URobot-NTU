import requests
import json
import cv2
import copy
import numpy as np


class Skeleton:
    def __init__(self):
        self.url = "https://api-cn.faceplusplus.com/humanbodypp/v1/skeleton"

        self.api_key = 'CpNmTaD49EUIyUQ00CYioMYyJaNpyGbA'
        self.api_secret = 'PNTCQOjJZO6E82JTJP8uhQrPPvN4lWPs'
        self.image_file = 'temp.jpg'

    def skeleton_point(self, img):
        self.img = copy.copy(img)
        files = {"api_key": (None, self.api_key),
                 "api_secret": (None, self.api_secret),
                 "image_file": ("dancer", open(self.image_file, "rb"), "image/jpeg")}
        res = requests.post(url=self.url, files=files)
        output = json.loads(res.text)
        if not output['skeletons']:
            return self.img, 0
        body = output['skeletons'][0]
        self.body_pos = body['body_rectangle']
        self.ske_point = body['landmark']
        self._skeleton_draw()
        return self.img, 1

    def get_center(self):
        point_set = ['neck', 'left_shoulder', 'right_shoulder', 'left_buttocks', 'right_buttocks']
        temp = np.zeros((1, 3))
        for name in point_set:
            p = self.get_point(name)
            temp += p
        temp /= len(point_set)
        return int(temp[0][0]), int(temp[0][1]), temp[0][2]

    def get_point(self, name):
        point = self.ske_point[name]
        x = point['x'] + self.body_pos['left']
        y = point['y'] + self.body_pos['top']
        score = point['score']
        return [x, y, score]

    def _skeleton_draw(self):
        self._draw('head', 'neck')
        self._draw('neck', 'left_shoulder')
        self._draw('neck', 'right_shoulder')
        self._draw('left_shoulder', 'left_elbow')
        self._draw('right_shoulder', 'right_elbow')
        self._draw('left_elbow', 'left_hand')
        self._draw('right_elbow', 'right_hand')
        self._draw('left_shoulder', 'left_buttocks')
        self._draw('right_shoulder', 'right_buttocks')
        self._draw('left_buttocks', 'right_buttocks')
        self._draw('left_buttocks', 'left_knee')
        self._draw('right_buttocks', 'right_knee')
        self._draw('left_knee', 'left_foot')
        self._draw('right_knee', 'right_foot')
        x, y, _ = self.get_center()
        cv2.circle(self.img, (x, y), 7, (0, 255, 0))

    def _draw(self, front, back):
        point1 = self.ske_point[front]
        x1 = point1['x'] + self.body_pos['left']
        y1 = point1['y'] + self.body_pos['top']
        cv2.circle(self.img, (x1, y1), 3, (255, 0, 0))
        point2 = self.ske_point[back]
        x2 = point2['x'] + self.body_pos['left']
        y2 = point2['y'] + self.body_pos['top']
        cv2.circle(self.img, (x2, y2), 3, (255, 0, 0))
        cv2.line(self.img, (x1, y1), (x2, y2), (255, 0, 0), thickness=3)











