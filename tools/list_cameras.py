def get_windows_camera_list():
    """
    获取 Windows 系统下所有摄像头的名称、唯一 ID 和 OpenCV 索引
    """
    try:
        import cv2
        import wmi
    except ImportError as exc:
        raise RuntimeError(
            "摄像头枚举工具需要可选依赖 opencv-python 和 WMI。"
        ) from exc

    c = wmi.WMI()

    # 查找所有属于 'Video' 或 'Image' 类别的 PnP 设备
    # 常见的摄像头在 Win32_PnPEntity 中的类 GUID 通常包含摄像头的特征
    wmi_cameras = []
    for device in c.Win32_PnPEntity():
        # 过滤包含摄像头特征的设备
        # 注意：不同驱动显示的 Service 可能不同，这里通过类名或名称过滤比较通用
        if device.PNPClass in ["Image", "Camera", "Video"]:
            wmi_cameras.append({
                "Name": device.Name,
                "ID": device.DeviceID  # 这是硬件层面的唯一标识符
            })

    if not wmi_cameras:
        print("未检测到摄像头设备。")
        return []

    # OpenCV 的索引顺序通常与系统枚举顺序一致
    # 我们通过尝试打开每个索引来验证
    valid_cameras = []
    index = 0
    max_tests = 10  # 假设你不会插超过 10 个摄像头

    found_count = 0
    while found_count < len(wmi_cameras) and index < max_tests:
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW) # 使用 DirectShow 后端
        if cap.isOpened():
            # 记录当前索引对应的设备信息
            # 这里的映射是基于“顺序一致性”的假设
            info = wmi_cameras[found_count]
            valid_cameras.append({
                "cv_id": index,
                "name": info["Name"],
                "unique_id": info["ID"]
            })
            cap.release()
            found_count += 1
        index += 1

    return valid_cameras

def main() -> int:
    """Print the Windows camera inventory and return a process exit code."""

    print("正在扫描系统摄像头...\n")
    try:
        cameras = get_windows_camera_list()
    except RuntimeError as exc:
        print(f"[ERROR] {exc}")
        return 2

    if cameras:
        print(f"{'OpenCV ID':<10} | {'设备名称':<30} | {'唯一硬件 ID (DeviceID)'}")
        print("-" * 80)
        for cam in cameras:
            print(f"{cam['cv_id']:<10} | {cam['name']:<30} | {cam['unique_id']}")
    else:
        print("未找到可用摄像头。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
