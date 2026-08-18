# 结构化世界模型实验工作区

本目录是硕士课题的独立实验区，不修改现有机械臂主链路。当前版本完成了第一阶段最小闭环：

1. 定义语义、几何、机器人、标定和任务状态；
2. 定义候选抓取动作；
3. 使用可解释规则预测成功率、碰撞风险和综合不确定性；
4. 输出 `execute`、`probe`、`reobserve` 或 `reject` 决策；
5. 原子保存世界状态，并按试验目录记录预测和结果。

## 目录结构

```text
world_model_experiment/
├── config/default.json          规则模型阈值与权重
├── world_model/                 实验核心包
├── tests/                       单元测试
├── run_demo.py                  无硬件演示入口
├── data/                        运行后产生的世界状态
└── trials/                      运行后产生的试验日志
```

## 运行演示

在项目根目录执行：

```powershell
python .\world_model_experiment\run_demo.py
```

使用项目真实标定文件、合成RGB-D输入和主项目控制器Dry-run接口运行集成演示：

```powershell
python .\world_model_experiment\run_integration_demo.py
```

集成演示会验证：真实标定坐标转换、现有项目规划结果适配、Dry-run机器人状态读取、多个抓取候选生成、现有位姿安全检查和世界模型决策。它不会打开相机、连接机械臂或发送运动命令。

演示不会连接真实机械臂，也不会发送运动指令。运行后会生成：

```text
world_model_experiment/data/latest_world_state.json
world_model_experiment/trials/<trial_id>/
```

## 运行测试

```powershell
python -m unittest discover -v -s .\world_model_experiment\tests -p "test_*.py"
```

## 真机前准备

离线预检（不会连接任何设备）：

```powershell
.\world_model_experiment\tomorrow_tools.bat preflight
```

现场真实相机只读采集（不会连接机械臂）：

```powershell
$env:QWEN_API_KEY="你的密钥"
.\world_model_experiment\tomorrow_tools.bat camera "抓取红色杯子"
```

现场机械臂只读状态检查（不会使能、移动或控制夹爪）：

```powershell
.\world_model_experiment\tomorrow_tools.bat robot 127.0.0.1
```

`tomorrow_tools.bat`会调用`tomorrow_tools.ps1`，并自动绕过本机只针对当前进程的PowerShell脚本限制。工具专门使用包含Orbbec与乐白SDK的Python 3.11环境，在执行前强制设置`LEBAI_DRY_RUN=1`、清除`LEBAI_ALLOW_REAL_MOTION`。它没有真实运动模式。

完整现场顺序见 `TOMORROW_REAL_MACHINE_CHECKLIST_CN.md`。

## 当前模型边界

当前 `RuleWorldModel` 是世界模型 V0，用于验证数据结构、安全决策和日志链路。它不是论文最终的学习型转移模型。后续应按以下顺序推进：

1. 通过 Web 后端只读接口采集真实 Qwen 与 RGB-D 规划结果；
2. 为每次候选动作保存 `(state, action, prediction, outcome)`；
3. 收集真机与 dry-run 数据；
4. 使用 XGBoost 训练 `P(success | state, action)`；
5. 保留相同接口，将 `RuleWorldModel` 替换为学习型模型；
6. 增加抓取后视觉验证与失败恢复。

## 安全原则

本目录当前只进行离线决策。任何接入真机的代码必须继续遵守项目已有的工作空间、IK、奇异点、运动互斥、探测锁定和显式真机授权机制。世界模型的概率输出不能绕过硬安全约束。
