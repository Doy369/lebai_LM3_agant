from __future__ import annotations

import base64
import json
import math
import os
import re
import socket
import urllib.error
import urllib.request
from urllib.parse import urlparse
from dataclasses import replace
from typing import Any, Dict, List, Optional, Sequence

from .types import AgentDecision, AgentRuntimeConfig, GraspPose, ImageTargetPlan, ObjectCandidate, Pose3D


class QwenPickAgent:
    """
    模块二：Qwen 大模型代理。

    支持两种模式：
    1. 候选物体模式：根据结构化候选物体列表选目标。
    2. 图像模式：直接让 Qwen3.5-Plus 查看当前 RGB 图像，返回目标框。
    """

    def __init__(self, config: Optional[AgentRuntimeConfig] = None) -> None:
        self.config = config or self.from_env()

    @classmethod
    def from_env(cls) -> "QwenPickAgent":
        api_key = os.getenv("QWEN_API_KEY") or os.getenv("DASHSCOPE_API_KEY")
        model = os.getenv("QWEN_MODEL", "qwen3.5-plus")
        base_url = os.getenv(
            "QWEN_BASE_URL",
            "https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
        timeout_sec = float(os.getenv("QWEN_TIMEOUT_SEC", "30"))
        temperature = float(os.getenv("QWEN_TEMPERATURE", "0.1"))
        top_p = float(os.getenv("QWEN_TOP_P", "0.8"))
        max_tokens = int(os.getenv("QWEN_MAX_TOKENS", "700"))
        enable_model = os.getenv("QWEN_ENABLE_MODEL", "1") != "0"
        enable_rule_fallback = os.getenv("QWEN_ENABLE_RULE_FALLBACK", "1") != "0"

        config = AgentRuntimeConfig(
            api_key=api_key,
            model=model,
            base_url=base_url.rstrip("/"),
            timeout_sec=timeout_sec,
            temperature=temperature,
            top_p=top_p,
            max_tokens=max_tokens,
            enable_model=enable_model,
            enable_rule_fallback=enable_rule_fallback,
        )
        return cls(config=config)

    def plan_pick(
        self,
        user_command: str,
        candidates: List[ObjectCandidate],
        scene_context: Optional[Dict[str, Any]] = None,
    ) -> AgentDecision:
        if not candidates:
            return AgentDecision(
                success=False,
                selected_object_id=None,
                selected_object_name=None,
                grasp_pose=None,
                reason="视觉模块未提供任何候选物体，无法生成抓取决策。",
                execution_summary="未执行抓取。",
                candidate_count=0,
                source="rule",
            )

        model_error: Optional[str] = None
        if self._should_use_model():
            try:
                model_response = self._post_chat_completion(
                    messages=self._build_candidate_messages(user_command, candidates, scene_context or {}),
                    response_format={"type": "json_object"},
                )
                decision = self._parse_model_decision(
                    raw_text=model_response,
                    candidates=candidates,
                )
                decision.candidate_count = len(candidates)
                decision.source = "qwen"
                decision.raw_model_response = model_response
                return decision
            except Exception as exc:
                model_error = str(exc)

        if not self.config.enable_rule_fallback:
            return AgentDecision(
                success=False,
                selected_object_id=None,
                selected_object_name=None,
                grasp_pose=None,
                reason=f"Qwen 调用失败，且已禁用规则回退：{model_error or '未知错误'}",
                execution_summary="未执行抓取。",
                candidate_count=len(candidates),
                source="qwen",
                raw_model_response=model_error,
            )

        decision = self._rule_based_decision(
            user_command=user_command,
            candidates=candidates,
            scene_context=scene_context or {},
        )
        if model_error:
            decision.metadata["model_error"] = model_error
        return decision

    def plan_pick_from_image(
        self,
        user_command: str,
        image_bytes: bytes,
        image_mime_type: str = "image/jpeg",
        scene_context: Optional[Dict[str, Any]] = None,
        image_size_wh: Optional[Sequence[int]] = None,
    ) -> ImageTargetPlan:
        """
        让 Qwen 直接查看当前图像并返回目标框。
        """
        if not self._should_use_model():
            raise RuntimeError("未配置 QWEN_API_KEY / DASHSCOPE_API_KEY，无法执行图像抓取规划。")

        model_response = self._post_chat_completion(
            messages=self._build_image_messages(
                user_command=user_command,
                image_bytes=image_bytes,
                image_mime_type=image_mime_type,
                scene_context=scene_context or {},
            ),
            response_format={"type": "json_object"},
        )
        plan = self._parse_image_plan(model_response, image_size_wh=image_size_wh)
        plan.raw_model_response = model_response
        return plan

    def _should_use_model(self) -> bool:
        return bool(self.config.enable_model and self.config.api_key)

    def _post_chat_completion(
        self,
        messages: List[Dict[str, Any]],
        response_format: Optional[Dict[str, Any]] = None,
    ) -> str:
        url = f"{self.config.base_url}/chat/completions"
        payload: Dict[str, Any] = {
            "model": self.config.model,
            "temperature": self.config.temperature,
            "top_p": self.config.top_p,
            "max_tokens": self.config.max_tokens,
            "messages": messages,
        }
        if response_format is not None:
            payload["response_format"] = response_format

        request = urllib.request.Request(
            url=url,
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.config.api_key}",
            },
            method="POST",
        )

        try:
            with urllib.request.urlopen(request, timeout=self.config.timeout_sec) as response:
                data = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"Qwen HTTPError {exc.code}: {detail}") from exc
        except urllib.error.URLError as exc:
            hint = self._network_error_hint(exc)
            raise RuntimeError(f"Qwen 网络调用失败: {exc}{hint}") from exc

        try:
            return data["choices"][0]["message"]["content"]
        except Exception as exc:
            raise RuntimeError(f"Qwen 返回格式异常: {data}") from exc

    def _network_error_hint(self, error: urllib.error.URLError) -> str:
        host = urlparse(self.config.base_url).hostname or ""
        parts = [f" base_url={self.config.base_url}"]

        resolved_ip = ""
        try:
            resolved_ip = socket.gethostbyname(host)
        except Exception:
            resolved_ip = ""
        if resolved_ip:
            parts.append(f", resolved_ip={resolved_ip}")

        reason_text = str(getattr(error, "reason", error))
        host_lower = host.lower()
        if (
            host_lower == "dashscope.aliyuncs.com"
            and resolved_ip.startswith("198.18.")
        ) or "10061" in reason_text or "unexpected eof" in reason_text.lower():
            parts.append(
                "。当前环境更像是代理/TUN/Clash 对 dashscope.aliyuncs.com 的拦截或 fake-ip 干扰。"
                "可尝试将该域名改为 DIRECT，或改用对应区域的可达 base_url。"
            )

        return "".join(parts)

    def _build_candidate_messages(
        self,
        user_command: str,
        candidates: List[ObjectCandidate],
        scene_context: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        candidate_payload = [item.to_prompt_dict() for item in candidates]
        system_prompt = (
            "你是机器人抓取任务的决策代理。"
            "你必须从给定候选物体里选择最符合自然语言指令的目标，"
            "不能虚构不存在的物体，也不能修改给定坐标。"
            "请始终输出 JSON 对象，不要输出 Markdown。"
            "如果没有合适目标，请返回 success=false 并解释原因。"
        )

        user_payload = {
            "task": user_command,
            "scene_context": scene_context,
            "candidates": candidate_payload,
            "output_schema": {
                "success": "bool",
                "selected_object_id": "string or null",
                "selected_object_name": "string or null",
                "reason": "string",
                "execution_summary": "string",
                "matched_filters": ["string", "..."],
                "grasp_pose": {
                    "position": {"x": "float", "y": "float", "z": "float"},
                    "roll": "float",
                    "pitch": "float",
                    "yaw": "float",
                    "approach_vector": {"x": "float", "y": "float", "z": "float"},
                    "pre_grasp_offset_m": "float",
                },
            },
        }

        return [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": json.dumps(user_payload, ensure_ascii=False, indent=2)},
        ]

    def _build_image_messages(
        self,
        user_command: str,
        image_bytes: bytes,
        image_mime_type: str,
        scene_context: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        image_b64 = base64.b64encode(image_bytes).decode("ascii")
        system_prompt = (
            "你是机器人抓取系统的视觉决策代理。"
            "你会看到当前 RGB 图像和一条自然语言命令。"
            "你的任务是找出最应该抓取的目标，并返回严格 JSON。"
            "不要输出 Markdown，不要解释额外内容。"
            "如果图中没有符合要求的目标，请返回 success=false。"
        )

        instruction = {
            "task": user_command,
            "scene_context": scene_context,
            "requirements": [
                "只返回严格 JSON",
                "bbox_xyxy 使用当前图像的像素坐标 [x1, y1, x2, y2]",
                "center_uv 使用目标框中心像素 [u, v]",
                "如果目标不存在，success=false 且 bbox_xyxy/center_uv 设为 null",
            ],
            "output_schema": {
                "success": "bool",
                "selected_object_name": "string or null",
                "reason": "string",
                "bbox_xyxy": "[int, int, int, int] or null",
                "center_uv": "[int, int] or null",
                "confidence": "float",
            },
        }

        return [
            {"role": "system", "content": system_prompt},
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": json.dumps(instruction, ensure_ascii=False, indent=2),
                    },
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:{image_mime_type};base64,{image_b64}"},
                    },
                ],
            },
        ]

    def _parse_model_decision(
        self,
        raw_text: str,
        candidates: List[ObjectCandidate],
    ) -> AgentDecision:
        payload = self._extract_json(raw_text)
        candidate_map = {item.object_id: item for item in candidates}

        selected_object_id = payload.get("selected_object_id")
        if selected_object_id and selected_object_id not in candidate_map:
            raise ValueError(f"模型返回了未知 object_id: {selected_object_id}")

        selected_candidate = candidate_map.get(selected_object_id) if selected_object_id else None
        grasp_pose = self._parse_grasp_pose(payload.get("grasp_pose"), selected_candidate)
        selected_name = None
        if selected_candidate is not None:
            selected_name = selected_candidate.display_name or selected_candidate.name
        elif payload.get("selected_object_name"):
            selected_name = str(payload.get("selected_object_name"))

        success = bool(payload.get("success"))
        reason = str(payload.get("reason", "")).strip() or "Qwen 未提供原因。"
        summary = str(payload.get("execution_summary", "")).strip() or "Qwen 未提供执行摘要。"
        matched_filters = payload.get("matched_filters") or []

        return AgentDecision(
            success=success,
            selected_object_id=selected_object_id,
            selected_object_name=selected_name,
            grasp_pose=grasp_pose,
            reason=reason,
            execution_summary=summary,
            matched_filters=[str(item) for item in matched_filters],
        )

    def _parse_image_plan(
        self,
        raw_text: str,
        image_size_wh: Optional[Sequence[int]] = None,
    ) -> ImageTargetPlan:
        payload = self._extract_json(raw_text)
        success = bool(payload.get("success"))
        reason = str(payload.get("reason", "")).strip() or "Qwen 未提供原因。"
        name = payload.get("selected_object_name")
        selected_object_name = None if name is None else str(name)
        confidence = float(payload.get("confidence", 0.0) or 0.0)

        bbox = payload.get("bbox_xyxy")
        center = payload.get("center_uv")

        bbox_xyxy: Optional[List[int]] = None
        center_uv: Optional[List[int]] = None

        if bbox is not None:
            if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
                raise ValueError(f"Qwen 返回的 bbox_xyxy 非法: {bbox}")
            bbox_xyxy = [int(round(float(item))) for item in bbox]
            x1, y1, x2, y2 = bbox_xyxy
            if x2 <= x1 or y2 <= y1:
                raise ValueError(f"Qwen 返回的 bbox_xyxy 无效: {bbox_xyxy}")

        if center is not None:
            if not isinstance(center, (list, tuple)) or len(center) != 2:
                raise ValueError(f"Qwen 返回的 center_uv 非法: {center}")
            center_uv = [int(round(float(center[0]))), int(round(float(center[1])))]

        if image_size_wh is not None and len(image_size_wh) == 2:
            width = int(image_size_wh[0])
            height = int(image_size_wh[1])
            if bbox_xyxy is not None:
                x1, y1, x2, y2 = bbox_xyxy
                if x1 < 0 or y1 < 0 or x2 > width or y2 > height:
                    raise ValueError(
                        f"Qwen 返回的 bbox_xyxy 超出图像范围: bbox={bbox_xyxy}, image={width}x{height}"
                    )
            if center_uv is not None:
                u, v = center_uv
                if u < 0 or v < 0 or u >= width or v >= height:
                    raise ValueError(
                        f"Qwen 返回的 center_uv 超出图像范围: center={center_uv}, image={width}x{height}"
                    )

        return ImageTargetPlan(
            success=success,
            selected_object_name=selected_object_name,
            reason=reason,
            bbox_xyxy=bbox_xyxy,
            center_uv=center_uv,
            confidence=confidence,
        )

    def _extract_json(self, raw_text: str) -> Dict[str, Any]:
        text = raw_text.strip()
        if text.startswith("```"):
            match = re.search(r"```(?:json)?\s*(\{.*\})\s*```", text, flags=re.DOTALL)
            if match:
                text = match.group(1).strip()

        try:
            return json.loads(text)
        except json.JSONDecodeError:
            match = re.search(r"(\{.*\})", text, flags=re.DOTALL)
            if not match:
                raise ValueError(f"无法从模型响应中提取 JSON: {raw_text}")
            return json.loads(match.group(1))

    def _parse_grasp_pose(
        self,
        payload: Optional[Dict[str, Any]],
        candidate: Optional[ObjectCandidate],
    ) -> Optional[GraspPose]:
        if payload is None:
            return candidate.grasp_pose if candidate is not None else None

        position_payload = payload.get("position") or {}
        position = Pose3D(
            x=float(position_payload["x"]),
            y=float(position_payload["y"]),
            z=float(position_payload["z"]),
        )
        approach_payload = payload.get("approach_vector") or {"x": 0.0, "y": 0.0, "z": -1.0}
        approach = Pose3D(
            x=float(approach_payload["x"]),
            y=float(approach_payload["y"]),
            z=float(approach_payload["z"]),
        )
        return GraspPose(
            position=position,
            roll=float(payload.get("roll", 0.0)),
            pitch=float(payload.get("pitch", math.pi)),
            yaw=float(payload.get("yaw", 0.0)),
            approach_vector=approach,
            pre_grasp_offset_m=float(payload.get("pre_grasp_offset_m", 0.08)),
        )

    def _rule_based_decision(
        self,
        user_command: str,
        candidates: List[ObjectCandidate],
        scene_context: Dict[str, Any],
    ) -> AgentDecision:
        normalized_command = self._normalize_text(user_command)
        matched_filters: List[str] = []
        filtered = list(candidates)

        name_filtered = [item for item in filtered if self._matches_name_or_tag(normalized_command, item)]
        if name_filtered:
            filtered = name_filtered
            matched_filters.append("name_or_tag")

        color_filtered = [
            item for item in filtered if item.color and self._normalize_text(item.color) in normalized_command
        ]
        if color_filtered:
            filtered = color_filtered
            matched_filters.append("color")

        if not filtered:
            best = max(candidates, key=lambda item: item.confidence)
            return self._build_success_decision(
                candidate=best,
                matched_filters=["fallback_confidence"],
                reason="没有找到完全匹配的名称/颜色条件，已回退到置信度最高的候选物体。",
                source="rule",
                candidate_count=len(candidates),
                metadata={"scene_context": scene_context},
            )

        if "最近" in user_command or "nearest" in normalized_command or "closest" in normalized_command:
            best = min(filtered, key=lambda item: self._distance_xy(item.position))
            matched_filters.append("nearest")
        elif "最左" in user_command or "leftmost" in normalized_command:
            best = min(filtered, key=lambda item: item.position.y)
            matched_filters.append("leftmost")
        elif "最右" in user_command or "rightmost" in normalized_command:
            best = max(filtered, key=lambda item: item.position.y)
            matched_filters.append("rightmost")
        elif "最前" in user_command or "frontmost" in normalized_command:
            best = max(filtered, key=lambda item: item.position.x)
            matched_filters.append("frontmost")
        else:
            best = max(filtered, key=lambda item: item.confidence)
            matched_filters.append("highest_confidence")

        reason = (
            f"根据规则匹配从 {len(candidates)} 个候选中筛出 {len(filtered)} 个目标，"
            f"最终选择 object_id={best.object_id}。"
        )
        return self._build_success_decision(
            candidate=best,
            matched_filters=matched_filters,
            reason=reason,
            source="rule",
            candidate_count=len(candidates),
            metadata={"scene_context": scene_context},
        )

    def _build_success_decision(
        self,
        candidate: ObjectCandidate,
        matched_filters: List[str],
        reason: str,
        source: str,
        candidate_count: int,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> AgentDecision:
        grasp_pose = candidate.grasp_pose or self._default_grasp_pose(candidate)
        display_name = candidate.display_name or candidate.name
        summary = (
            f"选择抓取 {display_name}，"
            f"Base 坐标为 ({candidate.position.x:.4f}, {candidate.position.y:.4f}, {candidate.position.z:.4f}) m。"
        )
        return AgentDecision(
            success=True,
            selected_object_id=candidate.object_id,
            selected_object_name=display_name,
            grasp_pose=grasp_pose,
            reason=reason,
            execution_summary=summary,
            matched_filters=matched_filters,
            candidate_count=candidate_count,
            source=source,
            metadata=metadata or {},
        )

    def _default_grasp_pose(self, candidate: ObjectCandidate) -> GraspPose:
        return GraspPose(
            position=replace(candidate.position),
            roll=0.0,
            pitch=math.pi,
            yaw=0.0,
            approach_vector=Pose3D(0.0, 0.0, -1.0),
            pre_grasp_offset_m=0.08,
        )

    def _matches_name_or_tag(self, normalized_command: str, candidate: ObjectCandidate) -> bool:
        aliases = candidate.metadata.get("aliases", []) if isinstance(candidate.metadata, dict) else []
        terms = [
            candidate.name,
            candidate.display_name or "",
            *(candidate.tags or []),
            *aliases,
        ]
        normalized_terms = [self._normalize_text(term) for term in terms if term]
        return any(term and term in normalized_command for term in normalized_terms)

    def _normalize_text(self, text: str) -> str:
        return re.sub(r"\s+", "", text.strip().lower())

    def _distance_xy(self, pose: Pose3D) -> float:
        return math.sqrt(pose.x * pose.x + pose.y * pose.y)


def build_demo_candidates() -> List[ObjectCandidate]:
    """提供一个演示用候选目标列表，便于独立测试模块二。"""
    return [
        ObjectCandidate(
            object_id="obj_apple_red_01",
            name="apple",
            display_name="红苹果",
            color="red",
            position=Pose3D(0.42, -0.08, 0.12),
            confidence=0.98,
            tags=["fruit", "redapple", "苹果"],
        ),
        ObjectCandidate(
            object_id="obj_banana_01",
            name="banana",
            display_name="香蕉",
            color="yellow",
            position=Pose3D(0.35, 0.10, 0.11),
            confidence=0.95,
            tags=["fruit", "banana", "香蕉"],
        ),
        ObjectCandidate(
            object_id="obj_cup_blue_01",
            name="cup",
            display_name="蓝色杯子",
            color="blue",
            position=Pose3D(0.28, 0.22, 0.10),
            confidence=0.89,
            tags=["cup", "杯子", "container"],
        ),
    ]


if __name__ == "__main__":
    agent = QwenPickAgent.from_env()
    decision = agent.plan_pick(
        user_command="抓取红色的苹果",
        candidates=build_demo_candidates(),
        scene_context={"robot_state": "idle", "camera_frame": "base"},
    )
    print(json.dumps(decision.to_dict(), ensure_ascii=False, indent=2))
