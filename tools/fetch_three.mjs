import { writeFile } from 'node:fs/promises';
const files = {
  'build/three.module.js': 'three.module.js',
  'examples/jsm/loaders/GLTFLoader.js': 'GLTFLoader.js',
  'examples/jsm/controls/OrbitControls.js': 'OrbitControls.js',
  'examples/jsm/utils/BufferGeometryUtils.js': 'BufferGeometryUtils.js',
  LICENSE: 'LICENSE-three.txt',
};
await Promise.all(Object.entries(files).map(async ([path, name]) => {
  const response = await fetch(`https://unpkg.com/three@0.165.0/${path}`);
  if (!response.ok) throw Error(`${path}: ${response.status}`);
  const source = (await response.text()).replaceAll("from 'three'", "from './three.module.js'")
    .replaceAll("from '../utils/BufferGeometryUtils.js'", "from './BufferGeometryUtils.js'");
  await writeFile(new URL(`../robot_system/web/static/robot3d/vendor/${name}`, import.meta.url), source);
  console.log(name);
}));
