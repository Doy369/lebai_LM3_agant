"""Download the official OpenCV Zoo model; verify its Git LFS SHA-256."""
import hashlib
from pathlib import Path
import urllib.request

DESTINATION = Path(__file__).resolve().parents[1] / 'models' / 'object_detection_yolox_2022nov.onnx'
SHA256 = 'c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063'
URL = 'https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/object_detection_yolox/object_detection_yolox_2022nov.onnx'

def main():
    DESTINATION.parent.mkdir(parents=True, exist_ok=True)
    if DESTINATION.is_file() and hashlib.sha256(DESTINATION.read_bytes()).hexdigest() == SHA256:
        print('Local scene detector model ready.'); return
    with urllib.request.urlopen(URL, timeout=60) as response:
        data = response.read()
    if hashlib.sha256(data).hexdigest() != SHA256:
        raise RuntimeError('Model SHA-256 mismatch; download was not saved.')
    temporary = DESTINATION.with_suffix('.download')
    temporary.write_bytes(data)
    temporary.replace(DESTINATION)
    print('Local scene detector model ready.')

if __name__ == '__main__':
    main()
