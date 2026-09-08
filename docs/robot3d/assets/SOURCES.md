# Local 3D resources

Retrieved 2026-09-07. No production deployment was performed.

## Lebai_LM3.glb

- User-supplied repository: https://github.com/Doy369/lebai_LM3_agant
- Exact revision: `54d4ce8e4a6149326bc1d604a11b1f33d529aa73`
- Source path: `docs/models/Lebai_LM3.glb`
- SHA-256: `dc4f5b2a2e599d696498b5e44ec5964b945644495eb43b9816ad8ca858e7ec4f`
- Size: 3,376,080 bytes; Blender glTF exporter 4.4.56, glTF 2.0.
- Original reference: https://dtsci.cn/robot/lebai/ (inspected rendered page and existing source snapshot under `artifacts/lebai_model_review_20260907/online_page.html`). Direct binary download from that host failed with TLS connection closure; the user supplied the GitHub source instead. Byte identity with the currently hosted dtsci binary was **not** verified.
- No LICENSE file or license metadata was found in the supplied repository/model. The model is included for this user's requested local integration; no open-source license, redistribution right or third-party authorship is asserted. Confirm original model rights before redistributing or publishing it. This notice is not a replacement license.
- `model-inspection.json` is generated from this exact binary by `node tools/inspect_robot_model.mjs`; dimensions are mesh coordinates, not measured robot dimensions.

## Three.js

Vendored version **0.165.0** from `https://unpkg.com/three@0.165.0/` (the official npm package distribution): `build/three.module.js`, `examples/jsm/loaders/GLTFLoader.js`, `examples/jsm/controls/OrbitControls.js`, `examples/jsm/utils/BufferGeometryUtils.js`.

MIT license and copyright retained in `../vendor/LICENSE-three.txt`. Only import paths in the three addon files were changed to local relative paths. No external CDN, Draco worker or decoder is used at runtime. `tools/fetch_three.mjs` reproduces the addon downloads/import rewrites. No source from the unlicensed reference page was copied wholesale.
