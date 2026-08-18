# 第一阶段兼容说明

本阶段只调整目录、入口和路径依赖，不改变标定数学、抓取流程、HTTP 接口或安全状态机。

| 旧入口 | 推荐入口 |
| --- | --- |
| `python .\01_collect_data.py` | `lebai-collect-calibration` |
| `python .\02_calibrate.py` | `lebai-solve-calibration` |
| `python .\03_module2_qwen_demo.py` | `lebai-qwen-demo` |
| `python .\04_module1_module2_pipeline_demo.py` | `lebai-vision-demo` |
| `python .\05_module3_controller_demo.py` | `lebai-controller-demo` |
| `python .\06_full_pick_pipeline_demo.py` | `lebai-pick-demo` |
| `python .\07_real_robot_safety_probe_demo.py` | `lebai-safety-probe` |
| `python .\08_two_stage_real_robot_test.py` | `lebai-two-stage-test` |
| `python .\09_module4_fastapi_backend.py` | `lebai-web` |
| `python .\10_validate_eye_to_hand_with_measured_point.py` | `lebai-validate-calibration` |
| `python .\11_auto_calibration.py` | `lebai-auto-calibrate` |

旧入口暂时保留为不超过 20 行的包装器。第二阶段只有在回归测试、真机前检查和文档迁移全部稳定后，才会讨论是否移除这些兼容文件。
