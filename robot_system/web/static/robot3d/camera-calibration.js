import {Matrix4} from './vendor/three.module.js';
import {matrixFromRows, optical_T_threeCamera, scene_T_base} from './coordinates.js';
import {number, vector} from './validation.js';

export function validateImageSize(value, name = '图像尺寸') {
  const size = vector(value, 2, name, 1, 16384);
  if (!size.every(Number.isInteger)) throw Error(`${name}：宽高必须是整数`);
  return size;
}

export function validateCameraMatrix(value) {
  if (!Array.isArray(value) || value.length !== 3 || value.some(row => !Array.isArray(row) || row.length !== 3)) {
    throw Error('相机内参需要 3×3 矩阵');
  }
  const rows = value.map((row, i) => vector(row, 3, `相机内参第${i + 1}行`));
  if (rows[0][0] <= 0 || rows[1][1] <= 0) throw Error('相机焦距 fx/fy 必须大于 0');
  if (Math.abs(rows[0][1]) > 1e-9 || Math.abs(rows[1][0]) > 1e-9 ||
      Math.abs(rows[2][0]) > 1e-9 || Math.abs(rows[2][1]) > 1e-9 || Math.abs(rows[2][2] - 1) > 1e-9) {
    throw Error('当前虚拟相机仅支持无 skew 的标准针孔内参');
  }
  return rows;
}

export function scaledIntrinsics(cameraMatrix, sourceSizeWh, renderSizeWh) {
  const matrix = validateCameraMatrix(cameraMatrix);
  const [sourceWidth, sourceHeight] = validateImageSize(sourceSizeWh, '标定图像尺寸');
  const [renderWidth, renderHeight] = validateImageSize(renderSizeWh, '虚拟渲染尺寸');
  const scaleX = renderWidth / sourceWidth;
  const scaleY = renderHeight / sourceHeight;
  return {
    fx: matrix[0][0] * scaleX,
    fy: matrix[1][1] * scaleY,
    cx: matrix[0][2] * scaleX,
    cy: matrix[1][2] * scaleY,
    width: renderWidth,
    height: renderHeight,
  };
}

export function projectionMatrixFromIntrinsics(intrinsics, near = 0.01, far = 30) {
  const fx = number(intrinsics?.fx, 'fx', Number.EPSILON);
  const fy = number(intrinsics?.fy, 'fy', Number.EPSILON);
  const cx = number(intrinsics?.cx, 'cx', 0, intrinsics?.width);
  const cy = number(intrinsics?.cy, 'cy', 0, intrinsics?.height);
  const width = number(intrinsics?.width, '图像宽度', 1);
  const height = number(intrinsics?.height, '图像高度', 1);
  const nearM = number(near, '近裁面', Number.EPSILON);
  const farM = number(far, '远裁面', nearM + Number.EPSILON);
  const left = -cx * nearM / fx;
  const right = (width - cx) * nearM / fx;
  const top = cy * nearM / fy;
  const bottom = -(height - cy) * nearM / fy;
  return new Matrix4().makePerspective(left, right, top, bottom, nearM, farM);
}

export function scene_T_calibratedCamera(baseTCameraRows) {
  return scene_T_base.clone()
    .multiply(matrixFromRows(baseTCameraRows))
    .multiply(optical_T_threeCamera);
}

export function calibratedCameraConfigFromApi(payload, renderSizeWh = [640, 360]) {
  const data = payload?.data ?? payload;
  if (!data || typeof data !== 'object') throw Error('标定接口没有返回数据');
  const distCoeffs = vector(data.dist_coeffs, data.dist_coeffs?.length ?? 0, '畸变系数');
  if (![4, 5, 8, 12, 14].includes(distCoeffs.length)) throw Error('畸变系数长度必须为 4/5/8/12/14');
  matrixFromRows(data.base_T_camera);
  return {
    mode: 'calibrated',
    imageSizeWh: validateImageSize(data.image_size_wh, '标定图像尺寸'),
    renderSizeWh: validateImageSize(renderSizeWh, '虚拟渲染尺寸'),
    cameraMatrix: validateCameraMatrix(data.camera_matrix),
    distCoeffs,
    baseTCamera: data.base_T_camera.map(row => [...row]),
    nearM: 0.01,
    farM: 30,
    calibrationFingerprint: String(data.calibration_fingerprint || data.path || 'unknown'),
    physicalValidation: 'pending',
  };
}

export function validateCalibratedCameraConfig(value) {
  if (value?.mode !== 'calibrated') throw Error('相机模式必须为 calibrated');
  const imageSizeWh = validateImageSize(value.imageSizeWh, '标定图像尺寸');
  const renderSizeWh = validateImageSize(value.renderSizeWh, '虚拟渲染尺寸');
  if (renderSizeWh[0] !== 640 || renderSizeWh[1] !== 360) throw Error('当前虚拟渲染目标固定为 640×360');
  const cameraMatrix = validateCameraMatrix(value.cameraMatrix);
  const distCoeffs = vector(value.distCoeffs, value.distCoeffs?.length ?? 0, '畸变系数');
  if (![4, 5, 8, 12, 14].includes(distCoeffs.length)) throw Error('畸变系数长度必须为 4/5/8/12/14');
  if (!Array.isArray(value.baseTCamera)) throw Error('缺少 base_T_camera');
  const baseTCamera = value.baseTCamera.map(row => Array.isArray(row) ? [...row] : row);
  matrixFromRows(baseTCamera);
  const nearM = number(value.nearM, '近裁面', 0.001, 1);
  const farM = number(value.farM, '远裁面', nearM + 0.001, 1000);
  if (!['pending', 'verified'].includes(value.physicalValidation)) throw Error('物理验证状态必须为 pending 或 verified');
  return {
    mode: 'calibrated', imageSizeWh, renderSizeWh, cameraMatrix, distCoeffs, baseTCamera,
    nearM, farM,
    calibrationFingerprint: String(value.calibrationFingerprint || 'unknown'),
    physicalValidation: value.physicalValidation,
  };
}
