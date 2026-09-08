import { Matrix4, Euler, Vector3, Quaternion } from './vendor/three.module.js';
import { vector } from './validation.js';

// A_T_B maps p_B to p_A. Matrices serialized row-major, translation in meters.
export function matrixFromRows(rows) {
  if (!Array.isArray(rows) || rows.length !== 4) throw Error('变换需要 4×4 矩阵');
  const values = rows.flatMap(row => vector(row, 4, '变换矩阵'));
  const m = new Matrix4().set(...values);
  if (values.slice(12).some((v, i) => Math.abs(v - [0, 0, 0, 1][i]) > 1e-8)) throw Error('变换末行无效');
  const axes = [0, 1, 2].map(i => new Vector3().setFromMatrixColumn(m, i));
  if (axes.some(a => Math.abs(a.length() - 1) > 1e-5) ||
      Math.abs(axes[0].dot(axes[1])) > 1e-5 || Math.abs(axes[1].dot(axes[2])) > 1e-5 ||
      Math.abs(axes[2].dot(axes[0])) > 1e-5 || Math.abs(m.determinant() - 1) > 1e-5) throw Error('变换旋转必须正交且 det=1');
  return m;
}
export const scene_T_base = new Matrix4().makeRotationX(-Math.PI / 2);
export const base_T_model = new Matrix4().makeRotationX(Math.PI / 2); // provisional glTF Y-up → Base Z-up
export const optical_T_threeCamera = new Matrix4().makeRotationX(Math.PI); // CV x right,y down,z forward
export function base_T_tcp(p) {
  const xyz = vector([p.x, p.y, p.z], 3, 'TCP(m)');
  const [rx, ry, rz] = vector([p.rx, p.ry, p.rz], 3, 'TCP(rad)');
  return new Matrix4().compose(new Vector3(...xyz), new Quaternion().setFromEuler(new Euler(rx, ry, rz, 'ZYX')), new Vector3(1, 1, 1));
}
