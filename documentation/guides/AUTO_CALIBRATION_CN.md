# 自动手眼标定使用说明

自动标定前请先把标定板刚性固定在夹爪上，并把机械臂移动到“棋盘格已经在相机画面中央附近”的安全初始姿态。

系统会围绕当前 TCP 姿态做小范围平移和倾角扰动，逐帧检查棋盘格，采集成功后直接调用 `robot_system.calibration.eye_to_hand_solver` 求解，不再依赖脚本文件名。

## 命令行运行

```powershell
Set-Location <project-directory>
$env:LEBAI_DRY_RUN="0"
$env:LEBAI_ROBOT_IP="127.0.0.1"
lebai-auto-calibrate --allow-real-motion --samples 20 --min-success 12 --settle-sec 0.8
```

只预览自动姿态计划，不运动：

```powershell
lebai-auto-calibrate --plan-only --samples 20
```

## Web 控制台

启动后端后打开：

```text
http://127.0.0.1:8001/ui/
```

页面里会出现“自动手眼标定”区域。建议先点“预览计划”，确认姿态数量和机器人状态正常后，再点“运行自动标定”。

## 安全注意

- 标定板必须刚性固定，不能只是放在夹爪上。
- 自动标定会真实移动机械臂，运行时人要守在急停旁。
- 命令行真机运行必须显式提供 `--allow-real-motion`；仅设置 `LEBAI_DRY_RUN=0` 不会开始运动。
- 新标定会先写入临时文件并完成矩阵校验，成功后才替换旧标定；失败时旧文件保持不变。
- 初始姿态必须让棋盘格处在相机画面中，否则有效样本会不足。
- 每个目标姿态都会先做工作空间、IK 和关节范围检查，失败样本会记录到 `calib_data/auto_*/rejected`。
