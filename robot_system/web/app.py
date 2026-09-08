from __future__ import annotations

import hmac
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Callable, Optional

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response
from fastapi.staticfiles import StaticFiles

from .models import (
    AutoCalibrationRunRequest,
    CameraGraspRequest,
    CameraPlanRequest,
    ExecutePickRequest,
    GenericMessageResponse,
    JogTCPRequest,
    PlanPickRequest,
    SafetyProbeRequestModel,
)
from .service import RobotWebService, WebBackendConfig


def create_app(config: Optional[WebBackendConfig] = None) -> FastAPI:
    """创建 FastAPI 应用。"""
    runtime_config = config or WebBackendConfig.from_env()
    service = RobotWebService(runtime_config)
    static_dir = Path(__file__).resolve().parent / "static"

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        try:
            yield
        finally:
            service.shutdown()

    app = FastAPI(
        title=runtime_config.app_title,
        version=runtime_config.app_version,
        description="乐白机械臂 + Orbbec 深度相机 + Qwen3.5-Plus 抓取系统后端接口",
        lifespan=lifespan,
    )
    app.state.service = service

    protected_paths = (
        "/api/robot/",
        "/api/pipeline/",
        "/api/calibration/auto/run",
    )

    @app.middleware("http")
    async def require_control_token(request: Request, call_next: Callable[..., object]) -> Response:
        if request.url.path.startswith(protected_paths):
            expected = runtime_config.web_token
            supplied = request.headers.get("X-Lebai-Token", "")
            if expected and not hmac.compare_digest(supplied, expected):
                return JSONResponse(status_code=401, content={"detail": "控制令牌无效或缺失。"})
        return await call_next(request)  # type: ignore[misc,no-any-return]

    app.add_middleware(
        CORSMiddleware,
        allow_origins=runtime_config.cors_origins,
        allow_credentials=False,
        allow_methods=["GET", "POST", "OPTIONS"],
        allow_headers=["Content-Type", "X-Lebai-Token"],
    )

    if static_dir.exists():
        app.mount("/ui", StaticFiles(directory=str(static_dir), html=True), name="ui")

    @app.get("/", response_model=GenericMessageResponse, tags=["system"])
    def root() -> dict:
        return {
            "success": True,
            "message": "Lebai Robot Web API 已启动。",
            "data": {
                "docs": "/docs",
                "openapi": "/openapi.json",
                "ui": "/ui",
            },
        }

    @app.get("/api/health", response_model=GenericMessageResponse, tags=["system"])
    def health() -> dict:
        return {
            "success": True,
            "message": "服务健康检查通过。",
            "data": service.health(),
        }

    @app.get("/api/config", response_model=GenericMessageResponse, tags=["system"])
    def runtime_config_view() -> dict:
        return service.runtime_config()

    @app.get("/api/examples/plan-pick", response_model=GenericMessageResponse, tags=["system"])
    def example_plan_pick() -> dict:
        return service.export_example_payload()

    @app.get("/api/examples/plan-from-camera", response_model=GenericMessageResponse, tags=["system"])
    def example_plan_from_camera() -> dict:
        return service.export_camera_example_payload()

    @app.get("/api/calibration/status", response_model=GenericMessageResponse, tags=["calibration"])
    def calibration_status() -> dict:
        return service.calibration_status()

    @app.get("/api/calibration/auto/plan", response_model=GenericMessageResponse, tags=["calibration"])
    def auto_calibration_plan() -> dict:
        return service.auto_calibration_plan()

    @app.post("/api/calibration/auto/run", response_model=GenericMessageResponse, tags=["calibration"])
    def run_auto_calibration(request: AutoCalibrationRunRequest) -> dict:
        return _call_service(lambda: service.run_auto_calibration(request))

    @app.get("/api/camera/status", response_model=GenericMessageResponse, tags=["camera"])
    def camera_status() -> dict:
        return service.camera_status()

    @app.get("/api/camera/orbbec-self-test", response_model=GenericMessageResponse, tags=["camera"])
    def camera_orbbec_self_test() -> dict:
        return service.orbbec_self_test()

    @app.get("/api/camera/frame.jpg", tags=["camera"])
    def camera_frame() -> Response:
        try:
            payload = service.camera_frame_jpeg()
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return Response(content=payload, media_type="image/jpeg")

    @app.get("/api/camera/latest-plan.jpg", tags=["camera"])
    def latest_plan_frame() -> Response:
        try:
            payload = service.latest_plan_frame_jpeg()
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        return Response(content=payload, media_type="image/jpeg")

    @app.get("/api/robot/status", response_model=GenericMessageResponse, tags=["robot"])
    def robot_status(response: Response) -> dict:
        response.headers["Cache-Control"] = "no-store"
        return _call_service(service.robot_status)

    @app.post("/api/robot/emergency-stop", response_model=GenericMessageResponse, tags=["robot"])
    def emergency_stop() -> dict:
        return _call_service(service.emergency_stop)

    @app.post("/api/robot/start-system", response_model=GenericMessageResponse, tags=["robot"])
    def start_robot_system() -> dict:
        return _call_service(service.start_robot_system)

    @app.post("/api/robot/stop-motion", response_model=GenericMessageResponse, tags=["robot"])
    def stop_motion() -> dict:
        return _call_service(service.stop_motion)

    @app.post("/api/robot/gripper/open", response_model=GenericMessageResponse, tags=["robot"])
    def open_gripper() -> dict:
        return _call_service(service.open_gripper)

    @app.post("/api/robot/gripper/close", response_model=GenericMessageResponse, tags=["robot"])
    def close_gripper() -> dict:
        return _call_service(service.close_gripper)

    @app.post("/api/robot/reset", response_model=GenericMessageResponse, tags=["robot"])
    def reset_robot() -> dict:
        return _call_service(service.reset_robot)

    @app.post("/api/robot/home/record", response_model=GenericMessageResponse, tags=["robot"])
    def record_home_pose() -> dict:
        return _call_service(service.record_home_pose)

    @app.post("/api/robot/teach-mode/enter", response_model=GenericMessageResponse, tags=["robot"])
    def enter_teach_mode() -> dict:
        return _call_service(service.enter_teach_mode)

    @app.post("/api/robot/teach-mode/exit", response_model=GenericMessageResponse, tags=["robot"])
    def exit_teach_mode() -> dict:
        return _call_service(service.exit_teach_mode)

    @app.post("/api/robot/jog-tcp", response_model=GenericMessageResponse, tags=["robot"])
    def jog_tcp(request: JogTCPRequest) -> dict:
        return _call_service(lambda: service.jog_tcp(request))

    @app.post("/api/agent/plan-pick", response_model=GenericMessageResponse, tags=["agent"])
    def plan_pick(request: PlanPickRequest) -> dict:
        return _call_service(lambda: service.plan_pick(request))

    @app.post("/api/agent/plan-from-camera", response_model=GenericMessageResponse, tags=["agent"])
    def plan_from_camera(request: CameraPlanRequest) -> dict:
        return _call_service(lambda: service.plan_from_camera(request))

    @app.get("/api/experiment/world-model/status", response_model=GenericMessageResponse, tags=["experiment"])
    def world_model_status() -> dict:
        return _call_service(service.world_model_status)

    @app.post("/api/experiment/world-model/plan-from-camera", response_model=GenericMessageResponse, tags=["experiment"])
    def world_model_plan_from_camera(request: CameraPlanRequest) -> dict:
        return _call_service(lambda: service.world_model_plan_from_camera(request))

    @app.post("/api/pipeline/probe", response_model=GenericMessageResponse, tags=["pipeline"])
    def safety_probe(request: SafetyProbeRequestModel) -> dict:
        return _call_service(lambda: service.safety_probe(request))

    @app.post("/api/pipeline/pick", response_model=GenericMessageResponse, tags=["pipeline"])
    def execute_pick(request: ExecutePickRequest) -> dict:
        return _call_service(lambda: service.execute_pick(request))

    @app.post("/api/pipeline/grasp-from-camera", response_model=GenericMessageResponse, tags=["pipeline"])
    def grasp_from_camera(request: CameraGraspRequest) -> dict:
        return _call_service(lambda: service.grasp_from_camera(request))

    return app


def _call_service(callback: Callable[[], dict]) -> dict:
    try:
        return callback()
    except PermissionError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except RuntimeError as exc:
        text = str(exc).lower()
        status_code = 503 if ("pyorbbecsdk" in text or "orbbec" in text) else 500
        raise HTTPException(status_code=status_code, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc
