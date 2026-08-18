# Lebai 机器人抓取系统使用说明

本文档用于说明当前项目在本机如何安装、如何验证、如何按顺序运行。

当前工程已经包含以下模块：

1. 标定数据采集：[`01_collect_data.py`](01_collect_data.py)
2. 眼在手外手眼标定：[`02_calibrate.py`](02_calibrate.py)
3. 大模型决策演示：[`03_module2_qwen_demo.py`](03_module2_qwen_demo.py)
4. 视觉坐标转换 + 大模型联调：[`04_module1_module2_pipeline_demo.py`](04_module1_module2_pipeline_demo.py)
5. 机械臂控制 dry-run 演示：[`05_module3_controller_demo.py`](05_module3_controller_demo.py)
6. 视觉到抓取全链路 dry-run：[`06_full_pick_pipeline_demo.py`](06_full_pick_pipeline_demo.py)
7. 真机安全探测脚本：[`07_real_robot_safety_probe_demo.py`](07_real_robot_safety_probe_demo.py)
8. 两阶段真机测试脚本：[`08_two_stage_real_robot_test.py`](08_two_stage_real_robot_test.py)
9. FastAPI 后端启动入口：[`09_module4_fastapi_backend.py`](09_module4_fastapi_backend.py)

## 1. 环境准备

建议环境：

- Windows 10/11
- Python 3.11
- 机械臂与电脑处于同一局域网
- Gemini RGB 相机可被系统识别

建议先确认 Python 版本：

```powershell
python --version
```

如果不是 Python 3.11，也可以继续使用 3.10 以上版本，但我当前这套代码是在 3.11 环境下做的语法检查。

## 2. 创建虚拟环境

在项目根目录执行：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
```

如果 PowerShell 禁止激活脚本，可以先执行：

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\.venv\Scripts\Activate.ps1
```

## 3. 需要安装哪些依赖

### 3.1 核心依赖

这些是当前项目最基础的运行依赖：

- `numpy`
- `opencv-python`

安装命令：

```powershell
pip install -r requirements-core.txt
```

对应文件：[`requirements-core.txt`](requirements-core.txt)

### 3.2 Web 后端依赖

如果你要启动模块四的 FastAPI 后端，还需要：

- `fastapi`
- `uvicorn`
- `pydantic`

安装命令：

```powershell
pip install -r requirements-web.txt
```

对应文件：[`requirements-web.txt`](requirements-web.txt)

### 3.3 可选工具依赖

这些不是主链必须依赖，但在某些调试场景有用：

- `pygrabber`
  用于按设备名枚举 Windows 摄像头，当前采集脚本默认并不强依赖它。
- `WMI`
  用于运行 [`camera.py`](camera.py) 扫描 Windows 摄像头信息。

安装命令：

```powershell
pip install -r requirements-optional.txt
```

如需运行项目单元测试，再安装开发依赖：

```powershell
pip install -r requirements-dev.txt
python -m unittest discover -v -s . -p "test*.py"
```

对应文件：[`requirements-optional.txt`](requirements-optional.txt)

### 3.4 机械臂 SDK

机械臂控制模块依赖 `lebai_sdk`，但这个包的分发方式可能因你拿到的官方版本而不同。

你需要安装：

- `lebai_sdk`

常见安装方式有两种：

1. 如果乐白官方提供 pip 安装方式，按官方方式安装。
2. 如果乐白官方提供的是本地 `whl` 文件，则在该文件所在目录执行：

```powershell
pip install .\你的_lebai_sdk.whl
```

安装完成后，建议执行：

```powershell
python -c "import lebai_sdk; print('lebai_sdk ok')"
```

如果这条命令报错，说明机械臂模块暂时还不能跑真机。

### 3.5 大模型接口

模块二调用 Qwen 时没有额外第三方 Python 包依赖，因为当前实现使用的是 Python 标准库 `urllib`。

但你需要准备好 API Key，并配置环境变量：

```powershell
$env:QWEN_API_KEY="你的APIKey"
```

可选环境变量：

```powershell
$env:QWEN_MODEL="qwen-plus"
$env:QWEN_BASE_URL="https://dashscope.aliyuncs.com/compatible-mode/v1"
```

如果你不配置 API Key，模块二仍然可以运行，但会自动回退到规则决策模式。

## 4. 推荐安装顺序

建议你按下面顺序安装和验证：

1. 创建虚拟环境并激活
2. 安装核心依赖
3. 安装 `lebai_sdk`
4. 如果要做 Web 后端，再安装 Web 依赖
5. 如果要用摄像头枚举工具，再安装可选依赖

推荐直接执行：

```powershell
pip install -r requirements-core.txt
pip install -r requirements-web.txt
pip install -r requirements-optional.txt
```

然后再单独安装乐白官方 SDK。

## 5. 如何验证是否安装成功

### 5.1 验证核心依赖

```powershell
python -c "import cv2, numpy; print('opencv', cv2.__version__); print('numpy ok')"
```

### 5.2 验证 Web 依赖

```powershell
python -c "import fastapi, uvicorn, pydantic; print('web deps ok')"
```

### 5.3 验证机械臂 SDK

```powershell
python -c "import lebai_sdk; print('lebai_sdk ok')"
```

### 5.4 验证当前工程模块能否正常导入

```powershell
python -c "from robot_system.pipeline import PickExecutor; print('project import ok')"
```

## 6. 运行顺序建议

当前项目建议按这个顺序使用：

### 第一步：先验证标定文件

确认以下文件存在：

- [`biaoding/eye_to_hand_result.json`](biaoding/eye_to_hand_result.json)

如果没有这个文件，先运行采集和标定：

```powershell
python .\01_collect_data.py
python .\02_calibrate.py
```

### 第二步：验证模块二

```powershell
python .\03_module2_qwen_demo.py
```

### 第三步：验证模块一 + 模块二

```powershell
python .\04_module1_module2_pipeline_demo.py
```

### 第四步：验证模块三 dry-run

```powershell
python .\05_module3_controller_demo.py
```

### 第五步：验证全链路 dry-run

```powershell
python .\06_full_pick_pipeline_demo.py
```

### 第六步：真机探测

先编辑 [`08_two_stage_real_robot_test.py`](08_two_stage_real_robot_test.py)，重点检查：

- `TEST_STAGE`
- `DRY_RUN`
- `ROBOT_IP`
- `WORKSPACE`
- `PROBE_DESCEND_CLEARANCE_M`

然后按顺序运行：

1. `TEST_STAGE="probe"` 且 `DRY_RUN=True`
2. `TEST_STAGE="probe"` 且 `DRY_RUN=False`
3. 确认探测位置无误后，再切换 `TEST_STAGE="pick"`

运行命令：

```powershell
python .\08_two_stage_real_robot_test.py
```

## 7. 模块四后端与控制台如何启动

先安装 Web 依赖：

```powershell
pip install -r requirements-web.txt
```

然后启动：

```powershell
python .\09_module4_fastapi_backend.py
```

默认地址：

- 后端首页：`http://127.0.0.1:8001/`
- Swagger 文档：`http://127.0.0.1:8001/docs`
- Web 控制台：`http://127.0.0.1:8001/ui`

说明：

- 当前控制台由 FastAPI 直接托管，不依赖 Node.js。
- 这样可以先把状态监控、控制面板和 AI 联调跑通。
- 如果后面你希望升级成独立 Vue3 工程，也可以在这个基础上继续拆分。

## 8. Web 后端可配置环境变量

### 机械臂与运行模式

```powershell
$env:LEBAI_ROBOT_IP="127.0.0.1"
$env:LEBAI_DRY_RUN="1"
```

真机模式必须同时显式确认并设置控制令牌，否则后端会拒绝启动：

```powershell
$env:LEBAI_DRY_RUN="0"
$env:LEBAI_ALLOW_REAL_MOTION="YES"
$env:LEBAI_WEB_TOKEN="请替换为足够长的随机字符串"
```

打开控制台后，将同一个令牌填入“控制令牌”输入框。令牌只保存在当前浏览器会话中。

### 后端监听地址

```powershell
$env:LEBAI_WEB_HOST="127.0.0.1"
$env:LEBAI_WEB_PORT="8001"
```

默认只监听本机。只有在已经配置防火墙、控制令牌和明确的 CORS 来源后，才应改成局域网监听地址。

### 安全探测锁

完整抓取前必须先执行一次“当前画面安全探测”。探测成功后产生一次性锁，默认 30 秒有效；锁过期、标定变化、机械臂位置偏移或已经消费时，完整抓取都会被拒绝。

```powershell
$env:LEBAI_PROBE_LOCK_TTL_SEC="30"
$env:LEBAI_PROBE_LOCK_JOINT_TOLERANCE_RAD="0.15"
```

基于手工 JSON 像素/深度数据的真机探测和抓取默认禁用。如确需现场调试，必须额外设置 `LEBAI_ALLOW_DEBUG_MOTION=1`。

### 相机预览

Web 控制台当前通过 OpenCV 的 `cv2.VideoCapture` 提供单帧 JPEG 预览，默认行为尽量与你采集脚本一致：

```powershell
$env:LEBAI_CAMERA_ID="0"
$env:LEBAI_CAMERA_BACKEND="auto"
$env:LEBAI_CAMERA_WIDTH="1280"
$env:LEBAI_CAMERA_HEIGHT="720"
$env:LEBAI_CAMERA_JPEG_QUALITY="90"
```

可选 backend：

- `auto`
- `dshow`
- `msmf`

如果控制台里看不到相机画面，可以按这个顺序排查：

1. 先确认没有别的程序占用 Gemini 相机
2. 保持 `LEBAI_CAMERA_ID=0`
3. 先试 `LEBAI_CAMERA_BACKEND=auto`
4. 如果还不行，再改成 `dshow` 或 `msmf`
5. 修改环境变量后，重启后端服务

### 标定文件路径

```powershell
$env:LEBAI_WEB_CALIBRATION_FILE="biaoding/eye_to_hand_result.json"
```

### 默认探测参数

```powershell
$env:LEBAI_DEFAULT_PROBE_DESCEND_CLEARANCE_M="0.05"
```

### 机器人 Home 位

如果你希望前端里的“复位”按钮可用，需要提供 6 个关节角：

```powershell
$env:LEBAI_HOME_JOINT_POSE="0,-1.2,1.3,0,1.57,0"
```

### 工作空间

```powershell
$env:LEBAI_WS_X_MIN="-0.75"
$env:LEBAI_WS_X_MAX="-0.15"
$env:LEBAI_WS_Y_MIN="-0.45"
$env:LEBAI_WS_Y_MAX="0.45"
$env:LEBAI_WS_Z_MIN="0.02"
$env:LEBAI_WS_Z_MAX="0.55"
```

### 默认放置点

如果你希望 Web 控制台和后端使用固定回收点，可以配置：

```powershell
$env:LEBAI_DROP_X="-0.30"
$env:LEBAI_DROP_Y="-0.22"
$env:LEBAI_DROP_Z="0.15"
$env:LEBAI_DROP_YAW="0.0"
$env:LEBAI_DROP_PRE_GRASP_OFFSET_M="0.10"
```

## 9. 常见问题

### 9.1 提示没有 `fastapi`

说明你还没有安装 Web 依赖，执行：

```powershell
pip install -r requirements-web.txt
```

### 9.2 提示没有 `lebai_sdk`

说明乐白官方 SDK 还没装好。当前项目里的真机动作模块无法绕过这个依赖。

### 9.3 相机打开成了笔记本摄像头

先确认 [`01_collect_data.py`](01_collect_data.py) 里的：

- `CAMERA_ID`
- `USE_CAMERA_NAME_MATCH`
- `CAMERA_BACKEND_MODE`

你当前这套代码默认更偏向直接使用：

```python
cv2.VideoCapture(CAMERA_ID)
```

如果是 Web 控制台相机预览，则对应的是这些环境变量：

- `LEBAI_CAMERA_ID`
- `LEBAI_CAMERA_BACKEND`
- `LEBAI_CAMERA_WIDTH`
- `LEBAI_CAMERA_HEIGHT`

### 9.4 真机读取状态时报连接中断

如果出现类似 `10054`、`restart required`、`远程主机强迫关闭连接` 这类错误，通常不是接口名错了，而是 SDK 连接已经被机器人端关闭了。当前控制器已经做了更清晰的报错和一次重连重试，但你仍然需要检查：

- 机械臂是否被其他程序占用
- 局域网是否稳定
- 机械臂是否处于急停或系统未启动状态

### 9.5 真机抓取时如何快速调参数

现在 Web 控制台已经支持直接调这些关键参数：

- 探测保留高度
- 探测悬停高度
- 抓取前悬停高度
- 抓取后抬升高度
- 放置前悬停高度
- 放置后撤离高度
- 放置点 X / Y / Z / Yaw

建议调试顺序：

1. 先用较大的探测保留高度做 `probe`
2. 对准后再减小探测保留高度
3. 再调抓取前后悬停与抬升
4. 最后再调放置点位置

## 10. 你现在最建议做什么

如果你准备继续推进，我建议按这个顺序：

1. 先把环境按本文档装齐
2. 跑一遍 `03 -> 04 -> 05 -> 06`
3. 再进行 `08` 的真机 `probe`
4. 真机探测稳定后，再继续做前端页面
