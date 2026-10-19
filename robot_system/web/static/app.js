const DEFAULT_LOCAL_API_BASE = "http://127.0.0.1:8001";

// Optional isolated viewer; a WebGL failure must not break existing controls.
import('./robot3d/index.js?v=20261013-image-objects2').then(({mountRobot3D}) => {
  mountRobot3D(document.getElementById('robot3d-panel'), {
    readStatus: signal => fetchJson('/api/robot/status', {method: 'GET', signal, cache: 'no-store'}),
    readCalibration: signal => fetchJson('/api/calibration/status', {method: 'GET', signal, cache: 'no-store'}),
    captureObjects: captureObjectsForVirtualScene,
    readCameraObjects: () => state.latestCameraCapture?.apiBase === state.apiBase ? state.latestCameraCapture : null,
    getCameraCommand: () => elements.commandInput.value.trim(),
  });
}).catch(error => {
  const panel = document.getElementById('robot3d-panel');
  if (panel) panel.textContent = `三维模块不可用：${error.message}。原有控制台可继续使用。`;
});

const state = {
  apiBase: resolveInitialApiBase(),
  controlToken: loadControlToken(),
  lastResult: null,
  latestCameraCapture: null,
  lastRobotStatus: null,
  logItems: [],
  chatItems: [],
  refreshTimer: null,
  cameraTimer: null,
  cameraStreamReady: false,
  cameraFramePending: false,
  runtimeConfig: null,
  probeLockId: null,
  probeLockSnapshotId: null,
  probeLockPlanId: null,
  previewedPlanId: null,
  virtualSceneRevision: 0,
  lastWorldModelStatus: null,
  lastWorldModelResult: null,
};

const elements = {
  apiBaseInput: document.getElementById("apiBaseInput"),
  controlTokenInput: document.getElementById("controlTokenInput"),
  backendStatusPill: document.getElementById("backendStatusPill"),
  robotStatusPill: document.getElementById("robotStatusPill"),
  robotSafetyPill: document.getElementById("robotSafetyPill"),
  healthSummary: document.getElementById("healthSummary"),
  healthDetail: document.getElementById("healthDetail"),
  calibrationSummary: document.getElementById("calibrationSummary"),
  calibrationDetail: document.getElementById("calibrationDetail"),
  robotSummary: document.getElementById("robotSummary"),
  robotDetail: document.getElementById("robotDetail"),
  cameraSummary: document.getElementById("cameraSummary"),
  cameraDetail: document.getElementById("cameraDetail"),
  cameraImage: document.getElementById("cameraImage"),
  cameraPlaceholderText: document.getElementById("cameraPlaceholderText"),
  planImage: document.getElementById("planImage"),
  planPlaceholderText: document.getElementById("planPlaceholderText"),
  resultSummary: document.getElementById("resultSummary"),
  resultDetail: document.getElementById("resultDetail"),
  snapshotDetail: document.getElementById("snapshotDetail"),
  eventLog: document.getElementById("eventLog"),
  chatFeed: document.getElementById("chatFeed"),
  commandInput: document.getElementById("commandInput"),
  detectionsInput: document.getElementById("detectionsInput"),
  sceneContextInput: document.getElementById("sceneContextInput"),
  probeClearanceInput: document.getElementById("probeClearanceInput"),
  probeHoverOffsetInput: document.getElementById("probeHoverOffsetInput"),
  autoRefreshToggle: document.getElementById("autoRefreshToggle"),
  pickPreOffsetInput: document.getElementById("pickPreOffsetInput"),
  pickPostOffsetInput: document.getElementById("pickPostOffsetInput"),
  placePreOffsetInput: document.getElementById("placePreOffsetInput"),
  placePostOffsetInput: document.getElementById("placePostOffsetInput"),
  dropXInput: document.getElementById("dropXInput"),
  dropYInput: document.getElementById("dropYInput"),
  dropZInput: document.getElementById("dropZInput"),
  dropYawInput: document.getElementById("dropYawInput"),
  autoCalibSampleCountInput: document.getElementById("autoCalibSampleCountInput"),
  autoCalibMinSuccessInput: document.getElementById("autoCalibMinSuccessInput"),
  autoCalibSettleInput: document.getElementById("autoCalibSettleInput"),
  autoCalibSessionInput: document.getElementById("autoCalibSessionInput"),
  autoCalibRunCalibrationToggle: document.getElementById("autoCalibRunCalibrationToggle"),
  worldModelStatusPill: document.getElementById("worldModelStatusPill"),
  worldModelMotionPill: document.getElementById("worldModelMotionPill"),
  wmDecisionMetric: document.getElementById("wmDecisionMetric"),
  wmSuccessMetric: document.getElementById("wmSuccessMetric"),
  wmUncertaintyMetric: document.getElementById("wmUncertaintyMetric"),
  wmCollisionMetric: document.getElementById("wmCollisionMetric"),
  wmTargetSummary: document.getElementById("wmTargetSummary"),
  wmTrialBadge: document.getElementById("wmTrialBadge"),
  wmCandidateTableBody: document.getElementById("wmCandidateTableBody"),
  wmPreflightChecks: document.getElementById("wmPreflightChecks"),
  wmRecentTrials: document.getElementById("wmRecentTrials"),
  wmDetail: document.getElementById("wmDetail"),
};

function init() {
  elements.apiBaseInput.value = state.apiBase;
  elements.controlTokenInput.value = state.controlToken;

  bindButton("refreshOverviewButton", refreshOverview);
  bindButton("refreshRobotButton", refreshRobotStatus);
  bindButton("refreshCalibrationButton", () => refreshCalibrationAndCamera({ refreshFrame: true }));
  bindButton("clearResultButton", clearResult);
  bindButton("clearLogButton", clearLogs);
  bindButton("loadExampleButton", loadDebugExamplePayload);
  bindButton("loadCameraExampleButton", loadCameraExamplePayload);
  bindButton("autoCalibrationPlanButton", previewAutoCalibrationPlan);
  bindButton("autoCalibrationRunButton", runAutoCalibration);
  bindButton("refreshWorldModelButton", refreshWorldModelStatus);
  bindButton("runWorldModelPlanButton", runWorldModelPlan);

  bindButton("startRobotButton", () => callRobotAction("/api/robot/start-system", {}, "启动机械臂"));
  bindButton("recordHomeButton", () => callRobotAction("/api/robot/home/record", {}, "记录当前位置为 Home"));
  bindButton("enterTeachModeButton", () => callRobotAction("/api/robot/teach-mode/enter", {}, "进入示教模式"));
  bindButton("exitTeachModeButton", () => callRobotAction("/api/robot/teach-mode/exit", {}, "退出示教模式"));
  bindButton("emergencyStopButton", () => callRobotAction("/api/robot/emergency-stop", {}, "急停"));
  bindButton("stopMotionButton", () => callRobotAction("/api/robot/stop-motion", {}, "停止运动"));
  bindButton("openGripperButton", () => callRobotAction("/api/robot/gripper/open", {}, "张开夹爪"));
  bindButton("closeGripperButton", () => callRobotAction("/api/robot/gripper/close", {}, "闭合夹爪"));
  bindButton("resetRobotButton", () => callRobotAction("/api/robot/reset", {}, "复位到 Home"));

  bindButton("planFromCameraButton", planFromCamera);
  bindButton("probeFromCameraButton", () => graspFromCamera("probe"));
  bindButton("pickFromCameraButton", () => graspFromCamera("pick"));
  bindButton("planPickButton", planPickFromDebugDetections);
  bindButton("probeButton", runProbeFromDebugDetections);
  bindButton("pickButton", runPickFromDebugDetections);

  elements.apiBaseInput.addEventListener("change", () => {
    state.apiBase = normalizeBaseUrl(elements.apiBaseInput.value);
    state.latestCameraCapture = null;
    elements.apiBaseInput.value = state.apiBase;
    saveApiBase(state.apiBase);
    addLog("系统", `已切换后端地址为 ${state.apiBase}`);
    refreshOverview().catch((error) => handleError("刷新总览", error));
  });

  elements.controlTokenInput.addEventListener("input", () => {
    state.controlToken = elements.controlTokenInput.value.trim();
    saveControlToken(state.controlToken);
    addLog("系统", state.controlToken ? "控制令牌已更新。" : "控制令牌已清除。");
  });

  elements.autoRefreshToggle.addEventListener("change", updateAutoRefresh);
  elements.commandInput.addEventListener("input", () => clearProbeLock("已修改抓取指令"));
  for (const element of [
    elements.pickPreOffsetInput, elements.pickPostOffsetInput,
    elements.placePreOffsetInput, elements.placePostOffsetInput,
    elements.dropXInput, elements.dropYInput, elements.dropZInput, elements.dropYawInput,
  ]) {
    element?.addEventListener("input", () => clearProbeLock("已修改统一抓取计划参数"));
  }
  window.addEventListener("lebai:compiled-plan-preview-complete", async (event) => {
    const planId = event.detail?.planId;
    if (planId && planId === state.probeLockPlanId) {
      const lockId = state.probeLockId, sceneRevision = state.virtualSceneRevision;
      try {
        await fetchJson("/api/pipeline/confirm-plan-preview", {
          method: "POST",
          body: JSON.stringify({probe_lock_id: lockId, plan_id: planId}),
        });
        if (state.probeLockId !== lockId || state.probeLockPlanId !== planId || state.virtualSceneRevision !== sceneRevision) return;
        state.previewedPlanId = planId;
        addLog("虚拟预演", `统一抓取计划 ${planId} 已完整播放并由后端登记；碰撞检查状态仍以计划验证字段为准。`);
        updateProbeAndPickGates();
        applyRobotSafetyState(state.lastRobotStatus);
      } catch (error) {
        handleError("确认统一计划预演", error);
      }
    }
  });
  window.addEventListener("lebai:virtual-scene-changed", () => {
    state.virtualSceneRevision++;
    state.previewedPlanId = null;
    updateProbeAndPickGates();
    applyRobotSafetyState(state.lastRobotStatus);
  });

  document.querySelectorAll("[data-jog]").forEach((button) => {
    button.addEventListener("click", () => runJog(button.dataset.jog, button.dataset.kind));
  });

  Promise.resolve()
    .then(loadCameraExamplePayload)
    .then(loadDebugExamplePayload)
    .then(refreshOverview)
    .then(updateAutoRefresh)
    .catch((error) => handleError("初始化", error));
}

function resolveInitialApiBase() {
  const queryApiBase = new URLSearchParams(window.location.search).get("apiBase");
  if (queryApiBase) {
    return normalizeBaseUrl(queryApiBase);
  }

  try {
    const savedApiBase = window.localStorage.getItem("lebai_api_base");
    if (savedApiBase) {
      return normalizeBaseUrl(savedApiBase);
    }
  } catch (error) {
    // Ignore storage failures in privacy-restricted browser contexts.
  }

  if (window.location.hostname.endsWith("github.io")) {
    return DEFAULT_LOCAL_API_BASE;
  }
  return `${window.location.protocol}//${window.location.host}`;
}

function saveApiBase(value) {
  try {
    window.localStorage.setItem("lebai_api_base", value);
  } catch (error) {
    // The input still works even when localStorage is unavailable.
  }
}

function loadControlToken() {
  try {
    return window.sessionStorage.getItem("lebai_control_token") || "";
  } catch (error) {
    return "";
  }
}

function saveControlToken(value) {
  try {
    if (value) {
      window.sessionStorage.setItem("lebai_control_token", value);
    } else {
      window.sessionStorage.removeItem("lebai_control_token");
    }
  } catch (error) {
    // The token remains usable for this page even when storage is unavailable.
  }
}

function normalizeBaseUrl(value) {
  const trimmed = String(value || "").trim();
  if (!trimmed) {
    return `${window.location.protocol}//${window.location.host}`;
  }
  return trimmed.replace(/\/+$/, "");
}

function bindButton(id, handler) {
  const button = document.getElementById(id);
  if (!button) return;
  button.addEventListener("click", async () => {
    button.disabled = true;
    try {
      await handler();
    } finally {
      button.disabled = false;
    }
  });
}

async function fetchJson(path, options = {}) {
  const response = await fetch(`${state.apiBase}${path}`, {
    ...options,
    headers: {
      "Content-Type": "application/json",
      ...(state.controlToken ? { "X-Lebai-Token": state.controlToken } : {}),
      ...(options.headers || {}),
    },
  });

  let payload = {};
  try {
    payload = await response.json();
  } catch {
    payload = {};
  }

  if (!response.ok) {
    const detail = payload.detail || payload.message || response.statusText;
    throw new Error(detail);
  }

  return payload;
}

async function refreshOverview() {
  await Promise.all([
    refreshHealth(),
    refreshRuntimeConfig(),
    refreshCalibrationAndCamera(),
    refreshRobotStatus(),
    refreshWorldModelStatus(),
  ]);
}

async function refreshWorldModelStatus() {
  const payload = await fetchJson("/api/experiment/world-model/status");
  const data = payload.data || {};
  state.lastWorldModelStatus = data;
  renderWorldModelStatus(data);
}

function renderWorldModelStatus(data) {
  const preflight = data.preflight || {};
  const checks = Array.isArray(preflight.checks) ? preflight.checks : [];
  const checksByName = Object.fromEntries(checks.map((item) => [item.name, item]));
  const runtimeSafety = data.runtime_safety || {};

  setPill(
    elements.worldModelStatusPill,
    preflight.failed ? "error" : preflight.ready_for_camera_capture ? "success" : "warning",
    preflight.failed ? "实验预检失败" : preflight.ready_for_camera_capture ? "感知实验可运行" : "实验准备未完成",
  );
  setPill(
    elements.worldModelMotionPill,
    runtimeSafety.world_model_plan_sends_motion ? "error" : "success",
    runtimeSafety.world_model_plan_sends_motion ? "警告：可能发送动作" : "世界模型只读规划",
  );

  setWorldModelGate("wmGateOffline", preflight.failed ? "fail" : "pass");
  const cameraCheck = checksByName.orbbec_sdk;
  const qwenCheck = checksByName.qwen_api_key;
  const cameraGate = preflight.ready_for_camera_capture
    ? "pass"
    : cameraCheck?.level === "fail"
      ? "fail"
      : "warn";
  setWorldModelGate("wmGateCamera", cameraGate);
  setWorldModelGate("wmGateRobot", preflight.ready_for_robot_readonly ? "pass" : "warn");
  setWorldModelGate("wmGateDryRun", data.latest_state ? "pass" : "pending");
  updateProbeAndPickGates();

  elements.wmPreflightChecks.innerHTML = checks.length
    ? checks
        .map(
          (item) => `
            <div class="check-item ${escapeHtml(item.level || "warn")}">
              <strong>${escapeHtml(preflightCheckLabel(item.name))}</strong>
              <span>${escapeHtml(item.level === "pass" ? "通过" : item.level === "fail" ? "失败" : "待处理")}</span>
            </div>
          `,
        )
        .join("")
    : '<span class="muted-text">暂无预检信息</span>';

  const trials = Array.isArray(data.recent_trials) ? data.recent_trials : [];
  elements.wmRecentTrials.innerHTML = trials.length
    ? trials
        .map(
          (trial) => `
            <div class="trial-item">
              <strong>${escapeHtml(trial.name || "unknown")}</strong>
              <span>${escapeHtml(formatTrialState(trial))}</span>
            </div>
          `,
        )
        .join("")
    : '<span class="muted-text">暂无实验记录</span>';

  if (!state.lastWorldModelResult && data.latest_state) {
    const targetId = data.latest_state?.task?.target_track_id || data.latest_state?.objects?.[0]?.track_id;
    elements.wmTargetSummary.textContent = targetId
      ? `最近世界状态：${targetId}`
      : "已读取最近世界状态";
    elements.wmDetail.textContent = prettyJson({
      preflight,
      runtime_safety: runtimeSafety,
      latest_state_path: data.latest_state_path,
      latest_state: data.latest_state,
    });
  }

  if (!qwenCheck || qwenCheck.level !== "pass") {
    elements.worldModelStatusPill.title = "配置 QWEN_API_KEY 或 DASHSCOPE_API_KEY 后才能运行真实画面分析。";
  } else {
    elements.worldModelStatusPill.title = "";
  }
}

async function runWorldModelPlan() {
  const userCommand = elements.commandInput.value.trim();
  if (!userCommand) throw new Error("请输入自然语言抓取指令。");
  const payload = {
    user_command: userCommand,
    scene_context: buildSceneContext(),
  };
  addChat("user", `${userCommand}（世界模型只读分析）`);
  addLog("世界模型", "正在采集真实RGB-D并进行Dry-run候选风险评价。不会发送机械臂动作。");
  const response = await fetchJson("/api/experiment/world-model/plan-from-camera", {
    method: "POST",
    body: JSON.stringify(payload),
  });
  state.lastWorldModelResult = response;
  renderWorldModelResult(response);
  consumeActionResult("世界模型只读分析", {
    ...response,
    data: response.data?.source_plan || response.data,
  });
  await refreshWorldModelStatus();
}

function renderWorldModelResult(response) {
  const data = response.data || {};
  const decision = data.world_model_decision || {};
  const prediction = decision.prediction || {};
  const target = data.target || {};
  const position = target.position_base_m || {};

  elements.wmDecisionMetric.textContent = worldModelActionLabel(decision.action);
  elements.wmSuccessMetric.textContent = formatPercent(prediction.success_probability);
  elements.wmUncertaintyMetric.textContent = formatPercent(prediction.total_uncertainty);
  elements.wmCollisionMetric.textContent = formatPercent(prediction.collision_probability);
  elements.wmTargetSummary.textContent = target.track_id
    ? `${target.name || "目标"} / ${target.track_id} / Base (${formatNumber(position.x)}, ${formatNumber(position.y)}, ${formatNumber(position.z)}) m`
    : "世界模型未构建出有效目标";
  elements.wmTrialBadge.textContent = data.trial_id || "未生成实验编号";
  elements.wmDetail.textContent = prettyJson(response);

  const candidates = Array.isArray(data.candidates) ? [...data.candidates] : [];
  candidates.sort((left, right) => {
    if (left.selected !== right.selected) return left.selected ? -1 : 1;
    return Number(right.prediction?.success_probability || 0) - Number(left.prediction?.success_probability || 0);
  });
  elements.wmCandidateTableBody.innerHTML = candidates.length
    ? candidates
        .map((item) => {
          const candidate = item.candidate || {};
          const candidatePrediction = item.prediction || {};
          return `
            <tr class="${item.selected ? "selected" : ""}">
              <td class="candidate-name">${escapeHtml(shortCandidateId(candidate.candidate_id))}</td>
              <td>${escapeHtml(candidate.source || "unknown")}</td>
              <td>${escapeHtml(formatPercent(candidatePrediction.success_probability))}</td>
              <td>${escapeHtml(formatPercent(candidatePrediction.collision_probability))}</td>
              <td>${escapeHtml(formatPercent(candidatePrediction.slip_probability))}</td>
              <td>${escapeHtml(formatPercent(candidatePrediction.total_uncertainty))}</td>
              <td><span class="candidate-status ${item.selected ? "selected" : ""}">${item.selected ? "已选择" : candidate.ik_reachable ? "可达" : "拒绝"}</span></td>
            </tr>
          `;
        })
        .join("")
    : '<tr><td colspan="7" class="empty-cell">没有生成可显示的抓取候选</td></tr>';

  setWorldModelGate("wmGateDryRun", response.success ? "pass" : "fail");
  setPill(
    elements.worldModelStatusPill,
    response.success ? "success" : "error",
    response.success ? `世界模型完成 / ${worldModelActionLabel(decision.action)}` : "世界模型分析失败",
  );
  refreshPlanImage();
}

function setWorldModelGate(id, kind) {
  const element = document.getElementById(id);
  if (!element) return;
  element.className = `gate-card ${kind}`;
}

function updateProbeAndPickGates() {
  setWorldModelGate("wmGateProbe", state.probeLockId ? "pass" : "pending");
  const previewPending = Boolean(state.probeLockPlanId && state.previewedPlanId !== state.probeLockPlanId);
  setWorldModelGate("wmGatePick", state.probeLockId && !previewPending ? "warn" : "locked");
}

function worldModelActionLabel(action) {
  const labels = {
    execute: "建议执行",
    probe: "建议先探测",
    reobserve: "需要重新观察",
    reject: "拒绝动作",
  };
  return labels[action] || "尚未决策";
}

function preflightCheckLabel(name) {
  const labels = {
    calibration: "标定",
    world_model_config: "模型配置",
    workspace: "工作空间",
    trial_output: "日志目录",
    free_disk: "磁盘空间",
    core_dependencies: "核心依赖",
    orbbec_sdk: "Orbbec SDK",
    lebai_sdk: "乐白 SDK",
    qwen_api_key: "Qwen Key",
    motion_environment: "运动环境",
  };
  return labels[name] || name || "未知检查";
}

function formatTrialState(trial) {
  if (trial.has_outcome) return "已有结果";
  if (trial.has_decision) return "已有决策";
  if (trial.has_world_state) return "已有状态";
  return "记录中";
}

function formatPercent(value) {
  const number = Number(value);
  return Number.isFinite(number) ? `${(number * 100).toFixed(1)}%` : "—";
}

function formatNumber(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(3) : "—";
}

function shortCandidateId(value) {
  const text = String(value || "unknown");
  return text.length > 30 ? `…${text.slice(-29)}` : text;
}

async function refreshHealth() {
  const payload = await fetchJson("/api/health");
  const data = payload.data || {};
  elements.healthSummary.textContent = `${data.service || "服务"} / ${data.version || "unknown"}`;
  elements.healthDetail.textContent = prettyJson(data);
  setPill(
    elements.backendStatusPill,
    data.calibration_file_exists ? "success" : "warning",
    data.dry_run ? "后端在线 / Dry Run" : data.read_only ? "后端在线 / 真机只读" : "后端在线 / 真机模式",
  );
}

async function refreshRuntimeConfig() {
  const payload = await fetchJson("/api/config");
  const data = payload.data || {};
  state.runtimeConfig = data;
  applyRobotSafetyState(state.lastRobotStatus);

  if (data.default_probe_descend_clearance_m != null) {
    elements.probeClearanceInput.value = Number(data.default_probe_descend_clearance_m).toFixed(3);
  }
  if (data.default_probe_hover_offset_m != null) {
    elements.probeHoverOffsetInput.value = Number(data.default_probe_hover_offset_m).toFixed(3);
  }

  const dropPose = data.drop_pose || {};
  const position = dropPose.position || {};
  if (position.x != null) elements.dropXInput.value = Number(position.x).toFixed(3);
  if (position.y != null) elements.dropYInput.value = Number(position.y).toFixed(3);
  if (position.z != null) elements.dropZInput.value = Number(position.z).toFixed(3);
  if (dropPose.yaw != null) elements.dropYawInput.value = Number(dropPose.yaw).toFixed(2);
  if (dropPose.pre_grasp_offset_m != null) {
    elements.placePreOffsetInput.value = Number(dropPose.pre_grasp_offset_m).toFixed(3);
  }
}

async function refreshCalibrationAndCamera({ refreshFrame = false } = {}) {
  const [calibrationPayload, cameraPayload] = await Promise.all([
    fetchJson("/api/calibration/status"),
    fetchJson("/api/camera/status"),
  ]);

  const calibration = calibrationPayload.data || {};
  const selectedMethod = calibration.selected_hand_eye_method || "未知";
  const samples = calibration.samples || {};
  elements.calibrationSummary.textContent = `方法 ${selectedMethod} / 样本 ${samples.used_count || 0}`;
  elements.calibrationDetail.textContent = prettyJson({
    path: calibration.path,
    board: calibration.board,
    image_size_wh: calibration.image_size_wh,
    selected_hand_eye_method: selectedMethod,
  });

  const camera = cameraPayload.data || {};
  const preview = camera.preview || {};
  const depthCamera = camera.depth_camera || {};
  state.cameraStreamReady = Boolean(preview.stream_ready);
  elements.cameraSummary.textContent = depthCamera.stream_ready
    ? "Orbbec RGB+Depth 已就绪"
    : preview.stream_ready
      ? "仅 RGB 预览已就绪"
      : "当前没有可用预览";
  elements.cameraDetail.textContent = prettyJson(camera);
  if (refreshFrame || elements.autoRefreshToggle.checked) {
    refreshCameraFrame();
  } else {
    const container = elements.cameraImage.parentElement;
    if (container) container.classList.remove("has-image");
    state.cameraFramePending = false;
    elements.cameraImage.removeAttribute("src");
    elements.cameraPlaceholderText.textContent = state.cameraStreamReady
      ? "相机已就绪；点击“刷新相机与标定”加载单帧预览"
      : "当前未检测到可用相机预览";
  }
}

async function refreshRobotStatus() {
  try {
    const payload = await fetchJson("/api/robot/status");
    const data = payload.data || {};
    state.lastRobotStatus = data;
    const kinData = data.kin_data || {};

    elements.robotSummary.textContent = data.dry_run
      ? "Dry Run 模式"
      : `${data.read_only ? "只读 / " : ""}${data.robot_state || "unknown"} / can_move=${Boolean(data.can_move)} / teach=${Boolean(data.teach_mode_requested)}`;
    elements.robotDetail.textContent = prettyJson({
      robot_state: data.robot_state,
      can_move: data.can_move,
      estop_reason: data.estop_reason,
      requires_manual_enable: data.requires_manual_enable,
      teach_mode_requested: data.teach_mode_requested,
      home_joint_pose: data.home_joint_pose,
      home_tcp_pose: data.home_tcp_pose,
      actual_joint_pose: kinData.actual_joint_pose,
      actual_tcp_pose: kinData.actual_tcp_pose,
      workspace: data.workspace,
      runtime_state_file: data.runtime_state_file,
    });

    setPill(
      elements.robotStatusPill,
      data.dry_run ? "warning" : data.can_move ? "success" : "error",
      data.dry_run ? "机器人 Dry Run" : data.read_only ? "真机只读同步" : data.can_move ? "机器人可运动" : "机器人不可运动",
    );

    const safetyText = data.dry_run
      ? "未连接真机"
      : data.read_only
        ? "只读模式 · 写请求已禁用"
      : data.teach_mode_requested
        ? "当前处于示教模式"
        : data.requires_manual_enable
          ? "请先启动并使能机械臂"
          : data.estop_reason
            ? `急停原因: ${data.estop_reason}`
            : "状态允许动作";
    const safetyKind = data.dry_run
      ? "warning"
      : data.teach_mode_requested || data.requires_manual_enable
        ? "warning"
        : "success";
    setPill(elements.robotSafetyPill, safetyKind, safetyText);
    applyRobotSafetyState(data);
  } catch (error) {
    setPill(elements.robotStatusPill, "error", "机器人状态失败");
    setPill(elements.robotSafetyPill, "warning", "无法判断运动安全");
    elements.robotSummary.textContent = "读取失败";
    elements.robotDetail.textContent = String(error.message || error);
    addLog("机器人状态", `读取失败: ${error.message || error}`, true);
    applyRobotSafetyState(null);
  }
}

async function loadDebugExamplePayload() {
  const payload = await fetchJson("/api/examples/plan-pick");
  const data = payload.data || {};
  clearProbeLock("已载入新的调试示例", { silent: true });
  if (!String(elements.commandInput.value || "").trim()) {
    elements.commandInput.value = data.user_command || "抓取红色杯子";
  }
  elements.detectionsInput.value = prettyJson(data.detections || []);
  if (!String(elements.sceneContextInput.value || "").trim()) {
    elements.sceneContextInput.value = prettyJson(data.scene_context || {});
  }
}

async function loadCameraExamplePayload() {
  const payload = await fetchJson("/api/examples/plan-from-camera");
  const data = payload.data || {};
  clearProbeLock("已载入新的当前画面示例", { silent: true });
  elements.commandInput.value = data.user_command || "抓取红色杯子";
  elements.sceneContextInput.value = prettyJson(data.scene_context || {});
  addLog("示例", "已载入当前画面抓取示例");
}

function parseJsonField(text, label, fallback) {
  const trimmed = String(text || "").trim();
  if (!trimmed) return fallback;
  try {
    return JSON.parse(trimmed);
  } catch (error) {
    throw new Error(`${label} 不是合法 JSON: ${error.message}`);
  }
}

function buildSceneContext() {
  return parseJsonField(elements.sceneContextInput.value, "场景上下文", {});
}

function buildDebugPayload() {
  return {
    user_command: elements.commandInput.value.trim(),
    detections: parseJsonField(elements.detectionsInput.value, "检测结果", []),
    scene_context: buildSceneContext(),
  };
}

function buildDropPoseOverride() {
  return {
    position: {
      x: Number(elements.dropXInput.value || -0.30),
      y: Number(elements.dropYInput.value || -0.22),
      z: Number(elements.dropZInput.value || 0.15),
    },
    roll: 0.0,
    pitch: Math.PI,
    yaw: Number(elements.dropYawInput.value || 0.0),
    approach_vector: { x: 0.0, y: 0.0, z: -1.0 },
    pre_grasp_offset_m: Number(elements.placePreOffsetInput.value || 0.1),
  };
}

function buildAutoCalibrationPayload() {
  return {
    sample_count: Number(elements.autoCalibSampleCountInput?.value || 20),
    min_success_samples: Number(elements.autoCalibMinSuccessInput?.value || 12),
    settle_sec: Number(elements.autoCalibSettleInput?.value || 0.8),
    execute_motion: true,
    run_calibration: Boolean(elements.autoCalibRunCalibrationToggle?.checked),
    auto_start_system: false,
    session_name: String(elements.autoCalibSessionInput?.value || "").trim() || null,
  };
}

async function previewAutoCalibrationPlan() {
  const response = await fetchJson("/api/calibration/auto/plan");
  consumeActionResult("自动标定计划", response);
}

async function runAutoCalibration() {
  const payload = buildAutoCalibrationPayload();
  addLog("自动标定", "自动采样开始。请保持标定板刚性固定，人在急停附近观察。");
  const response = await fetchJson("/api/calibration/auto/run", {
    method: "POST",
    body: JSON.stringify(payload),
  });
  consumeActionResult("自动手眼标定", response);
  await refreshOverview();
}

async function planFromCamera() {
  const sourceApiBase = state.apiBase;
  clearProbeLock("已执行新的当前画面规划");
  const payload = {
    user_command: elements.commandInput.value.trim(),
    scene_context: buildSceneContext(),
  };
  addChat("user", `${payload.user_command}（当前画面规划）`);
  const response = await fetchJson("/api/agent/plan-from-camera", {
    method: "POST",
    body: JSON.stringify(payload),
  });
  if (state.apiBase !== sourceApiBase) throw new Error("后端地址已更改，请重新识别当前画面");
  consumeActionResult("当前画面规划", response);
}

async function captureObjectsForVirtualScene(command, signal) {
  const sourceApiBase = state.apiBase;
  clearProbeLock("已开始新的相机识别");
  const response = await fetchJson(`/api/camera/objects?target=${encodeURIComponent(command)}`, {signal, cache: "no-store"});
  if (state.apiBase !== sourceApiBase) throw new Error("后端地址已更改，请重新识别当前画面");
  if (!response.success || !response.data?.objects?.length) throw Error(response.message || "未识别到可定位的物体");
  state.latestCameraCapture = {plan: response.data, apiBase: state.apiBase, capturedAt: response.data.capturedAt,
    photoUrl: `${state.apiBase}/api/camera/latest-objects.jpg?ts=${Date.now()}`};
  addLog("单张图像识别", response.message);
  return state.latestCameraCapture;
}

async function graspFromCamera(mode) {
  if (mode === "probe") {
    clearProbeLock("将以新的安全探测替换旧的探测锁", { silent: true });
  }
  if (mode === "pick" && !state.probeLockId) {
    throw new Error("完整抓取前必须先执行当前画面安全探测，并在探测锁有效期内抓取。");
  }
  if (mode === "pick" && state.probeLockPlanId && state.previewedPlanId !== state.probeLockPlanId) {
    throw new Error("完整抓取前必须先让三维场景完整播放本次统一抓取计划。请等待虚拟预演结束。");
  }
  const usingProbeLock = mode === "pick" && Boolean(state.probeLockId);
  const payload = {
    user_command: elements.commandInput.value.trim(),
    mode,
    scene_context: buildSceneContext(),
    descend_clearance_m: Number(elements.probeClearanceInput.value || 0.05),
    hover_offset_m: Number(elements.probeHoverOffsetInput.value || 0.08),
    drop_pose_override: buildDropPoseOverride(),
    pick_pre_grasp_offset_m: Number(elements.pickPreOffsetInput.value || 0.08),
    pick_post_grasp_offset_m: Number(elements.pickPostOffsetInput.value || 0.10),
    place_pre_place_offset_m: Number(elements.placePreOffsetInput.value || 0.10),
    place_post_place_offset_m: Number(elements.placePostOffsetInput.value || 0.10),
  };
  if (usingProbeLock) {
    payload.probe_lock_id = state.probeLockId;
  }
  addChat("user", `${payload.user_command}（${mode === "probe" ? "安全探测" : "完整抓取"}）`);
  try {
    const response = await fetchJson("/api/pipeline/grasp-from-camera", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    if (mode === "probe") {
      setProbeLockFromResponse(response);
      const compiledPlan = response.data?.compiled_plan;
      if (compiledPlan) window.dispatchEvent(new CustomEvent("lebai:compiled-plan", {
        detail: {plan: compiledPlan, trajectoryValidation: response.data?.trajectory_validation || null},
      }));
    } else if (usingProbeLock) {
      clearProbeLock("完整抓取已消费本次安全探测", { silent: true });
    }
    consumeActionResult(mode === "probe" ? "当前画面安全探测" : "当前画面完整抓取", response);
    await refreshRobotStatus();
  } catch (error) {
    if (mode === "pick" && usingProbeLock) {
      clearProbeLock("完整抓取失败，探测锁已失效", { silent: true });
    }
    throw error;
  }
}

async function planPickFromDebugDetections() {
  const payload = buildDebugPayload();
  addChat("user", `${payload.user_command}（JSON 调试规划）`);
  const response = await fetchJson("/api/agent/plan-pick", {
    method: "POST",
    body: JSON.stringify(payload),
  });
  consumeActionResult("JSON 调试规划", response);
}

async function runProbeFromDebugDetections() {
  const payload = {
    ...buildDebugPayload(),
    connect_robot: true,
    descend_clearance_m: Number(elements.probeClearanceInput.value || 0.05),
    hover_offset_m: Number(elements.probeHoverOffsetInput.value || 0.08),
  };
  const response = await fetchJson("/api/pipeline/probe", {
    method: "POST",
    body: JSON.stringify(payload),
  });
  consumeActionResult("JSON 调试安全探测", response);
  await refreshRobotStatus();
}

async function runPickFromDebugDetections() {
  const payload = {
    ...buildDebugPayload(),
    execute_motion: true,
    drop_pose_override: buildDropPoseOverride(),
    pick_pre_grasp_offset_m: Number(elements.pickPreOffsetInput.value || 0.08),
    pick_post_grasp_offset_m: Number(elements.pickPostOffsetInput.value || 0.10),
    place_pre_place_offset_m: Number(elements.placePreOffsetInput.value || 0.10),
    place_post_place_offset_m: Number(elements.placePostOffsetInput.value || 0.10),
  };
  const response = await fetchJson("/api/pipeline/pick", {
    method: "POST",
    body: JSON.stringify(payload),
  });
  consumeActionResult("JSON 调试完整抓取", response);
  await refreshRobotStatus();
}

async function callRobotAction(path, body, title) {
  const response = await fetchJson(path, {
    method: "POST",
    body: JSON.stringify(body || {}),
  });
  consumeActionResult(title, response);
  await refreshRobotStatus();
}

async function runJog(axis, kind) {
  const linearStep = Number(document.getElementById("linearStepInput").value || 0.01);
  const rotateStep = Number(document.getElementById("rotateStepInput").value || 0.05);
  const payload = {
    dx: 0.0,
    dy: 0.0,
    dz: 0.0,
    drz: 0.0,
    dry: 0.0,
    drx: 0.0,
    linear: true,
    label: `web_jog_${axis}`,
  };

  const amount = kind === "rotate" ? rotateStep : linearStep;
  if (axis === "x+") payload.dx = amount;
  if (axis === "x-") payload.dx = -amount;
  if (axis === "y+") payload.dy = amount;
  if (axis === "y-") payload.dy = -amount;
  if (axis === "z+") payload.dz = amount;
  if (axis === "z-") payload.dz = -amount;
  if (axis === "rz+") payload.drz = amount;
  if (axis === "rz-") payload.drz = -amount;

  const response = await fetchJson("/api/robot/jog-tcp", {
    method: "POST",
    body: JSON.stringify(payload),
  });
  consumeActionResult(`Jog ${axis}`, response);
  await refreshRobotStatus();
}

function consumeActionResult(title, response) {
  state.lastResult = response;
  const data = response.data || {};
  if (data.snapshot_id) {
    state.latestCameraCapture = response.success && data.success !== false && data.surface_point_base_m ? {
      plan: response, apiBase: state.apiBase, capturedAt: new Date().toISOString(),
      photoUrl: `${state.apiBase}/api/camera/latest-plan.jpg?ts=${Date.now()}`,
    } : null;
  }
  elements.resultSummary.textContent = `${title} / ${response.message || "完成"}`;
  elements.resultDetail.textContent = prettyJson(response);

  if (data.snapshot_id || data.center_uv || data.point_base_m || data.decision) {
    elements.snapshotDetail.textContent = prettyJson({
      snapshot_id: data.snapshot_id,
      lock_snapshot_id: data.lock_snapshot_id,
      probe_lock_id: data.probe_lock_id,
      execution_source: data.execution_source,
      bbox_xyxy: data.bbox_xyxy,
      center_uv: data.center_uv,
      depth_m: data.depth_m,
      point_base_m: data.point_base_m,
      grasp_pose: data.grasp_pose,
      compiled_plan_id: data.compiled_plan?.plan_id,
      trajectory_validation: data.trajectory_validation ? {
        report_id: data.trajectory_validation.report_id,
        plan_id: data.trajectory_validation.plan_id,
        kinematic_status: data.trajectory_validation.kinematic_status,
        collision_status: data.trajectory_validation.collision_status,
        gate_recommendation: data.trajectory_validation.gate_recommendation,
        sample_count: data.trajectory_validation.samples?.length,
        segments: data.trajectory_validation.segments,
        issues: data.trajectory_validation.issues,
        collision_validation: data.trajectory_validation.collision_validation ? {
          report_id: data.trajectory_validation.collision_validation.report_id,
          status: data.trajectory_validation.collision_validation.status,
          geometry_status: data.trajectory_validation.collision_validation.geometry_status,
          scene_status: data.trajectory_validation.collision_validation.scene_status,
          checked_sample_count: data.trajectory_validation.collision_validation.checked_sample_count,
          obstacle_count: data.trajectory_validation.collision_validation.obstacle_count,
          minimum_clearance_m: data.trajectory_validation.collision_validation.minimum_clearance_m,
          closest_pair: data.trajectory_validation.collision_validation.closest_pair,
          contacts: data.trajectory_validation.collision_validation.contacts,
          issues: data.trajectory_validation.collision_validation.issues,
        } : null,
      } : null,
      decision: data.decision,
      execution: data.execution,
    });
    refreshPlanImage();
  }

  addChat("assistant", `${title}：${response.message || "完成"}`);
  addLog(title, response.message || "完成", !response.success, response.data || response);
}

function addLog(title, message, isError = false, payload = null) {
  state.logItems.unshift({
    title,
    message,
    isError,
    payload,
    ts: new Date().toLocaleTimeString(),
  });
  state.logItems = state.logItems.slice(0, 50);
  renderLogs();
}

function addChat(role, message) {
  state.chatItems.unshift({
    role,
    message,
    ts: new Date().toLocaleTimeString(),
  });
  state.chatItems = state.chatItems.slice(0, 20);
  renderChat();
}

function renderLogs() {
  elements.eventLog.innerHTML = "";
  state.logItems.forEach((item) => {
    const wrapper = document.createElement("article");
    wrapper.className = "log-item";
    wrapper.innerHTML = `
      <div class="log-title">${escapeHtml(item.title)}</div>
      <div>${escapeHtml(item.message)}</div>
      <div class="log-meta">${escapeHtml(item.ts)}${item.isError ? " / 错误" : ""}</div>
      ${item.payload ? `<pre class="log-json">${escapeHtml(prettyJson(item.payload))}</pre>` : ""}
    `;
    elements.eventLog.appendChild(wrapper);
  });
}

function renderChat() {
  elements.chatFeed.innerHTML = "";
  state.chatItems.forEach((item) => {
    const wrapper = document.createElement("article");
    wrapper.className = "chat-item";
    wrapper.innerHTML = `
      <div class="chat-role">${item.role === "user" ? "你" : "系统"}</div>
      <div>${escapeHtml(item.message)}</div>
      <div class="log-meta">${escapeHtml(item.ts)}</div>
    `;
    elements.chatFeed.appendChild(wrapper);
  });
}

function clearResult() {
  state.lastResult = null;
  elements.resultSummary.textContent = "尚未执行动作";
  elements.resultDetail.textContent = "";
  elements.snapshotDetail.textContent = "";
}

function setProbeLockFromResponse(response) {
  const data = response?.data || {};
  if (!data.probe_lock_id) return;
  state.probeLockId = data.probe_lock_id;
  state.probeLockSnapshotId = data.lock_snapshot_id || data.snapshot_id || null;
  state.probeLockPlanId = data.compiled_plan?.plan_id || null;
  state.previewedPlanId = null;
  updateProbeAndPickGates();
  applyRobotSafetyState(state.lastRobotStatus);
  const snapshotLabel = state.probeLockSnapshotId || data.probe_lock_id;
  addChat("assistant", `完整抓取将复用本次安全探测：${snapshotLabel}`);
  addLog("安全探测锁", `完整抓取将复用本次安全探测：${snapshotLabel}`, false, {
    probe_lock_id: state.probeLockId,
    lock_snapshot_id: state.probeLockSnapshotId,
  });
}

function clearProbeLock(reason, options = {}) {
  if (!state.probeLockId) return;
  if (!options.silent) {
    addLog("安全探测锁", reason, false, {
      probe_lock_id: state.probeLockId,
      lock_snapshot_id: state.probeLockSnapshotId,
    });
  }
  state.probeLockId = null;
  state.probeLockSnapshotId = null;
  state.probeLockPlanId = null;
  state.previewedPlanId = null;
  updateProbeAndPickGates();
  applyRobotSafetyState(state.lastRobotStatus);
}

function clearLogs() {
  state.logItems = [];
  state.chatItems = [];
  renderLogs();
  renderChat();
}

function prettyJson(value) {
  if (value == null) return "";
  return JSON.stringify(value, null, 2);
}

function setPill(element, kind, text) {
  element.className = `status-pill ${kind}`;
  element.textContent = text;
}

function applyRobotSafetyState(robotStatus) {
  const requiresMotion = document.querySelectorAll("[data-requires-motion='true']");
  const readOnly = Boolean(state.runtimeConfig?.read_only || robotStatus?.read_only);
  const canMove = robotStatus ? Boolean(robotStatus.dry_run || robotStatus.can_move) : false;
  const teachModeRequested = robotStatus ? Boolean(robotStatus.teach_mode_requested) : false;
  const allowMotionActions = !readOnly && canMove && !teachModeRequested;
  const blockedReason = readOnly
    ? "后端处于真机只读模式，写请求已禁用"
    : teachModeRequested
    ? "当前处于示教模式，请先退出示教模式"
    : "当前 can_move=false，请先启动并使能机械臂";

  requiresMotion.forEach((element) => {
    element.disabled = !allowMotionActions;
    element.title = allowMotionActions ? "" : blockedReason;
  });

  const mainMotionButtons = [
    document.getElementById("probeFromCameraButton"),
    document.getElementById("pickFromCameraButton"),
  ];
  mainMotionButtons.forEach((button) => {
    if (!button) return;
    button.disabled = !allowMotionActions;
    button.title = allowMotionActions ? "" : blockedReason;
  });

  const pickFromCameraButton = document.getElementById("pickFromCameraButton");
  const previewPending = Boolean(state.probeLockPlanId && state.previewedPlanId !== state.probeLockPlanId);
  if (pickFromCameraButton && allowMotionActions && (!state.probeLockId || previewPending)) {
    pickFromCameraButton.disabled = true;
    pickFromCameraButton.title = !state.probeLockId
      ? "请先完成当前画面安全探测"
      : "请等待统一抓取计划完成虚拟播放";
  }

  if (readOnly) {
    for (const id of [
      "startRobotButton", "recordHomeButton", "enterTeachModeButton", "exitTeachModeButton",
      "emergencyStopButton", "stopMotionButton", "openGripperButton", "closeGripperButton",
      "resetRobotButton", "autoCalibrationRunButton", "planFromCameraButton",
      "probeFromCameraButton", "pickFromCameraButton", "planPickButton", "probeButton", "pickButton",
    ]) {
      const button = document.getElementById(id);
      if (button) { button.disabled = true; button.title = blockedReason; }
    }
  }
}

function refreshCameraFrame() {
  const container = elements.cameraImage.parentElement;
  if (!state.cameraStreamReady) {
    if (container) container.classList.remove("has-image");
    state.cameraFramePending = false;
    elements.cameraImage.removeAttribute("src");
    elements.cameraPlaceholderText.textContent = "当前未检测到可用相机预览";
    return;
  }
  // SDK 开流可能超过刷新间隔；等这一帧完成，避免反复取消加载。
  if (state.cameraFramePending) return;
  state.cameraFramePending = true;
  elements.cameraPlaceholderText.textContent = "正在读取相机画面…";
  elements.cameraImage.onload = () => {
    state.cameraFramePending = false;
    if (container) container.classList.add("has-image");
  };
  elements.cameraImage.onerror = () => {
    state.cameraFramePending = false;
    if (container) container.classList.remove("has-image");
    elements.cameraPlaceholderText.textContent = "实时相机画面读取失败";
  };
  elements.cameraImage.src = `${state.apiBase}/api/camera/frame.jpg?ts=${Date.now()}`;
}

function refreshPlanImage() {
  const container = elements.planImage.parentElement;
  elements.planImage.onload = () => {
    if (container) container.classList.add("has-image");
  };
  elements.planImage.onerror = () => {
    if (container) container.classList.remove("has-image");
    elements.planPlaceholderText.textContent = "最近一次抓取规划快照读取失败";
  };
  elements.planImage.src = `${state.apiBase}/api/camera/latest-plan.jpg?ts=${Date.now()}`;
}

function escapeHtml(text) {
  return String(text)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function handleError(title, error) {
  const message = error?.message || String(error);
  elements.resultSummary.textContent = `${title}失败`;
  elements.resultDetail.textContent = message;
  addChat("assistant", `${title}失败：${message}`);
  addLog(title, message, true);
}

function updateAutoRefresh() {
  if (state.refreshTimer) clearInterval(state.refreshTimer);
  if (state.cameraTimer) clearInterval(state.cameraTimer);
  state.refreshTimer = null;
  state.cameraTimer = null;

  if (elements.autoRefreshToggle.checked) {
    state.refreshTimer = setInterval(() => {
      refreshRobotStatus().catch((error) => handleError("自动刷新机器人状态", error));
    }, 3000);
    state.cameraTimer = setInterval(() => {
      refreshCameraFrame();
    }, 1200);
  }
}

window.addEventListener("error", (event) => {
  handleError("前端异常", event.error || event.message);
});

window.addEventListener("unhandledrejection", (event) => {
  handleError("异步请求异常", event.reason || "未知错误");
});

document.addEventListener("DOMContentLoaded", init);
