"""Local, single-image COCO detection and RGB-D projection; no robot control.

Model/preprocessing: OpenCV Zoo object_detection_yolox (Apache-2.0).
https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_yolox
"""
from pathlib import Path
import threading

import cv2
import numpy as np


CLASS_NAMES = (
    'person bicycle car motorcycle airplane bus train truck boat traffic_light '
    'fire_hydrant stop_sign parking_meter bench bird cat dog horse sheep cow '
    'elephant bear zebra giraffe backpack umbrella handbag tie suitcase frisbee '
    'skis snowboard sports_ball kite baseball_bat baseball_glove skateboard '
    'surfboard tennis_racket bottle wine_glass cup fork knife spoon bowl banana '
    'apple sandwich orange broccoli carrot hot_dog pizza donut cake chair couch '
    'potted_plant bed dining_table toilet tv laptop mouse remote keyboard '
    'cell_phone microwave oven toaster sink refrigerator book clock vase scissors '
    'teddy_bear hair_drier toothbrush'
).split()
LABELS = dict(zip(CLASS_NAMES, (
    '人 自行车 汽车 摩托车 飞机 公交车 火车 卡车 船 红绿灯 消防栓 停车标志 '
    '停车计时器 长椅 鸟 猫 狗 马 羊 牛 大象 熊 斑马 长颈鹿 背包 雨伞 手提包 '
    '领带 行李箱 飞盘 滑雪板 单板滑雪板 球 风筝 棒球棒 棒球手套 滑板 冲浪板 '
    '网球拍 瓶子 酒杯 杯子 叉子 刀 勺子 碗 香蕉 苹果 三明治 橙子 西兰花 胡萝卜 '
    '热狗 披萨 甜甜圈 蛋糕 椅子 沙发 盆栽 床 桌子 马桶 电视 笔记本电脑 鼠标 '
    '遥控器 键盘 手机 微波炉 烤箱 烤面包机 水槽 冰箱 书 时钟 花瓶 剪刀 '
    '泰迪熊 吹风机 牙刷'
).split()))
# Furniture, people and the robot environment are not movable scene objects.
SCENE_CLASSES = set(CLASS_NAMES[24:56] + CLASS_NAMES[58:59] + CLASS_NAMES[63:68] + CLASS_NAMES[73:])
PALETTE = {'banana': '#efc52d', 'orange': '#f19a35', 'apple': '#b8c952', 'broccoli': '#5b9d58', 'carrot': '#ed923e'}
MODEL_PATH = Path(__file__).resolve().parents[2] / 'models' / 'object_detection_yolox_2022nov.onnx'


class LocalSceneDetector:
    def __init__(self, model_path=MODEL_PATH):
        self.model_path = Path(model_path)
        self.net = None
        self.lock = threading.Lock()
        grids, strides = [], []
        for stride in (8, 16, 32):
            x, y = np.meshgrid(np.arange(640 // stride), np.arange(640 // stride))
            grids.append(np.stack((x, y), -1).reshape(-1, 2))
            strides.append(np.full((x.size, 1), stride))
        self.grids, self.strides = np.concatenate(grids), np.concatenate(strides)

    def detect(self, image, target=''):
        detections = self._detect_region(image, target)
        # Small tabletop objects need a larger image scale. Propose regions from
        # saturated foreground, then let the model classify each region.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 100, 80), (179, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w = image.shape[:2]
        for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:12]:
            if cv2.contourArea(contour) < 250:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            margin = max(bw, bh) * .6
            x1, y1 = max(0, int(x - margin)), max(0, int(y - margin))
            x2, y2 = min(w, int(x + bw + margin)), min(h, int(y + bh + margin))
            if (x2 - x1) * (y2 - y1) > w * h * .7:
                continue
            for detection in self._detect_region(image[y1:y2, x1:x2], target):
                a, b, c, d = detection['bbox_xyxy']
                detection['bbox_xyxy'] = [a + x1, b + y1, c + x1, d + y1]
                detections.append(detection)
        if not detections:
            return []
        boxes = [[a, b, c - a, d - b] for a, b, c, d in (o['bbox_xyxy'] for o in detections)]
        indices = cv2.dnn.NMSBoxesBatched(boxes, [o['confidence'] for o in detections],
                                        [CLASS_NAMES.index(o['className']) for o in detections], .35, .5)
        return [detections[int(i)] for i in np.asarray(indices).reshape(-1)][:20]

    def _detect_region(self, image, target=''):
        h, w = image.shape[:2]
        ratio = min(640 / h, 640 / w)
        padded = np.full((640, 640, 3), 114, dtype=np.float32)
        # YOLOX's non-legacy preprocessing keeps OpenCV's BGR channel order.
        resized = cv2.resize(image, (int(w * ratio), int(h * ratio)))
        padded[:resized.shape[0], :resized.shape[1]] = resized
        with self.lock:
            if self.net is None:
                if not self.model_path.is_file():
                    raise RuntimeError('缺少本地识别模型，请运行 tools/download_scene_detector.py。')
                # OpenCV's Windows filename reader cannot reliably open Chinese paths.
                self.net = cv2.dnn.readNetFromONNX(np.frombuffer(self.model_path.read_bytes(), dtype=np.uint8))
            self.net.setInput(padded.transpose(2, 0, 1)[None])
            output = self.net.forward()[0].copy()
        return self.decode(output, ratio, (w, h), target)

    def decode(self, output, ratio, image_size, target=''):
        output[:, :2] = (output[:, :2] + self.grids) * self.strides
        output[:, 2:4] = np.exp(output[:, 2:4]) * self.strides
        scores = output[:, 4:5] * output[:, 5:]
        classes = scores.argmax(1)
        confidence = scores.max(1)
        boxes = np.column_stack((output[:, :2] - output[:, 2:4] / 2, output[:, 2:4])) / ratio
        indices = cv2.dnn.NMSBoxesBatched(boxes.tolist(), confidence.tolist(), classes.tolist(), .35, .5)
        w, h = image_size
        target = target.strip().lower()
        requested = {name for name, label in LABELS.items() if label in target or name.replace('_', ' ') in target}
        result = []
        for i in sorted(np.asarray(indices).reshape(-1), key=lambda i: -confidence[i]):
            name = CLASS_NAMES[int(classes[i])]
            if name not in SCENE_CLASSES or (requested and name not in requested):
                continue
            x, y, bw, bh = boxes[i]
            bbox = [max(0, int(x)), max(0, int(y)), min(w, int(x + bw)), min(h, int(y + bh))]
            if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
                continue
            result.append({'className': name, 'label': LABELS[name], 'bbox_xyxy': bbox, 'confidence': float(confidence[i])})
        return result[:20]


def locate_objects(snapshot, transformer, detections):
    """Use the measured surface point, never the compensated robot flange pose."""
    transformer.assert_image_size(snapshot.image_size_wh)
    objects, skipped = [], []
    for detection in detections:
        x1, y1, x2, y2 = detection['bbox_xyxy']
        sample = snapshot.sample_depth_in_bbox((x1, y1, x2, y2))
        if sample is None:
            skipped.append({'label': detection['label'], 'reason': '没有有效深度'})
            continue
        depth = float(sample['depth_m'])
        position = transformer.pixel_to_base(u=sample['u'], v=sample['v'], depth=depth, depth_unit='m')
        corners = [transformer.pixel_to_base(u=u, v=v, depth=depth, depth_unit='m') for u, v in ((x1, y1), (x2, y1), (x1, y2), (x2, y2))]
        size = [float(np.clip(np.ptp([getattr(p, axis) for p in corners]), .02, .2)) for axis in ('x', 'y')]
        objects.append({**detection, 'id': f'R{len(objects) + 1}', 'positionBaseM': [position.x, position.y, position.z],
                        'pointKind': 'surface', 'sizeM': [*size, .04], 'yawRad': 0,
                        'color': PALETTE.get(detection['className'], '#35bca9'), 'depthM': depth,
                        'sizeSource': 'bbox approximation; height assumed 4 cm'})
    return objects, skipped
