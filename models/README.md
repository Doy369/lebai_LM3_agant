# 单张图片物体识别模型

模型为 [OpenCV Zoo 的 YOLOX](https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_yolox)，采用 Apache-2.0 许可。
模型文件不纳入 Git，缺失时运行 `python tools/download_scene_detector.py` 下载并校验 SHA-256。

此功能只拍摄一张 RGB-D 图片，在本机识别常见物体，再使用现有标定换算到机械臂基座坐标。
网页点击“识别当前画面物体”会自动生成虚拟物块，不需要开启视频自动刷新，不会发送机械臂动作。

相机使用的 Python 版本必须与 Orbbec SDK 一致；当前 `pyorbbecsdk.cp311` 需要 Python 3.11。
先安装 `requirements-core.txt`、`requirements-web.txt` 和对应版本的 Orbbec SDK，再下载上面的识别模型。

物体用长方体近似；平面尺寸根据检测框估计，高度暂按 4 cm，当前仿真支持 2～20 cm 的物块。
无法识别、没有有效深度或存在碰撞时会显示原因。这里只完成虚拟场景显示，尚未验证真机抓取精度。
