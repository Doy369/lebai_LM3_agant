"""
用一个“已知真实 Base 点”交叉验证眼在手外标定结果。

用途：
1. 你手动把夹爪尖端碰到桌面/工件某一点；
2. 记录乐白返回的 actual_tcp_pose.x/y/z；
3. 运行本脚本，检查该点在相机里应当投影到哪里。

如果投影位置与当前相机画面中的真实位置大体一致，说明 base_T_camera 基本可信；
如果偏得很离谱，就继续回头查手眼标定或工具坐标定义。
"""

from __future__ import annotations

from pathlib import Path

from robot_system.llm.types import Pose3D
from robot_system.vision.transformer import VisionTransformer


CALIBRATION_PATH = Path("biaoding/eye_to_hand_result.json")

# 把这里替换成你现场实测到的 Base 点。
# 当前默认填入的是你前面“夹爪碰到桌面”时读到的真实 TCP 位置。
MEASURED_BASE_POINT = Pose3D(
    x=-0.4443797350252131,
    y=0.007061958143565731,
    z=0.09249510380577718,
)


def main() -> int:
    transformer = VisionTransformer.from_calibration_file(CALIBRATION_PATH)

    point_camera = transformer.base_to_camera(MEASURED_BASE_POINT)
    u, v, depth_m = transformer.base_to_pixel(MEASURED_BASE_POINT)
    roundtrip_base = transformer.pixel_to_base(u=u, v=v, depth=depth_m, depth_unit="m")

    print("[INFO] 使用标定文件:", CALIBRATION_PATH)
    print("[INFO] 实测 Base 点:")
    print(
        f"       x={MEASURED_BASE_POINT.x:.6f} m, "
        f"y={MEASURED_BASE_POINT.y:.6f} m, "
        f"z={MEASURED_BASE_POINT.z:.6f} m"
    )
    print("[INFO] 反变换到相机坐标:")
    print(
        f"       Xc={point_camera.x:.6f} m, "
        f"Yc={point_camera.y:.6f} m, "
        f"Zc={point_camera.z:.6f} m"
    )
    print("[INFO] 投影到图像像素:")
    print(f"       u={u:.2f}, v={v:.2f}, depth={depth_m:.6f} m")
    print("[INFO] 再从像素 + 深度回到 Base 点:")
    print(
        f"       x={roundtrip_base.x:.6f} m, "
        f"y={roundtrip_base.y:.6f} m, "
        f"z={roundtrip_base.z:.6f} m"
    )
    print(
        "[INFO] 解释：如果上面的 (u, v) 大体落在你相机画面里对应的真实接触点附近，"
        "说明这份手眼外参基本自洽。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
