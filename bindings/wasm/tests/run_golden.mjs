// Replays common/geometry/golden/geometry_golden.json through the WASM build.
//
// This is the third runner over the same file -- the C++ one (tests/golden_ops.cpp)
// and the Python one (bindings/python/tests/test_golden.py) are the other two.
// Three languages agreeing on 269 cases at 1e-12 is the whole point of the golden
// file: a formula that drifts in one binding cannot hide.
//
// Usage:  node bindings/wasm/tests/run_golden.mjs [path/to/cyberwave_geometry.mjs]
// Build the module first with bindings/wasm/build.sh.

import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const modulePath = process.argv[2]
  ? resolve(process.argv[2])
  : resolve(here, "../dist/cyberwave_geometry.mjs");
const goldenPath = resolve(here, "../../../golden/geometry_golden.json");

const golden = JSON.parse(readFileSync(goldenPath, "utf8"));
const TOL = golden.tolerance;

// The module is built with ENVIRONMENT=web,worker (build.sh), so its loader
// fetch()es the .wasm -- which Node cannot do for a file:// URL. Handing the
// bytes over as `wasmBinary` skips that path entirely, so the artifact under
// test is the exact one the frontend ships rather than a Node-flavoured rebuild.
const { default: createGeometryModule } = await import(modulePath);
const geom = await createGeometryModule({
  wasmBinary: readFileSync(modulePath.replace(/\.mjs$/, ".wasm")),
});

// --- comparison ---------------------------------------------------------------

const failures = [];
let checked = 0;

function fail(id, message) {
  failures.push(`${id}: ${message}`);
}

// Relative above 1, absolute below. Mirrors `golden_budget` in
// tests/golden_ops.hpp and `_budget` in bindings/python/tests/test_golden.py.
function budget(expected) {
  return TOL * Math.max(1, Math.abs(expected));
}

function closeEnough(actual, expected) {
  return Number.isFinite(actual) && Math.abs(actual - expected) <= budget(expected);
}

function checkNumber(id, what, actual, expected) {
  checked += 1;
  if (!closeEnough(actual, expected)) {
    fail(id, `${what}: expected ${expected}, got ${actual} (delta ${Math.abs(actual - expected)})`);
  }
}

function checkFields(id, what, actual, expected, fields) {
  if (actual === undefined || actual === null) {
    fail(id, `${what}: missing`);
    return;
  }
  for (const field of fields) {
    checkNumber(id, `${what}.${field}`, actual[field], expected[field]);
  }
}

const checkQuat = (id, what, a, e) => checkFields(id, what, a, e, ["w", "x", "y", "z"]);
const checkVec = (id, what, a, e) => checkFields(id, what, a, e, ["x", "y", "z"]);
// Named fields, never three positional doubles: latitude read as longitude is a
// plausible place on Earth, which is exactly the failure this file has to catch.
const checkGeodetic = (id, what, a, e) =>
  checkFields(id, what, a, e, ["latitude", "longitude", "altitude"]);

function checkTransform(id, what, actual, expected) {
  if (actual === undefined || actual === null) {
    fail(id, `${what}: missing`);
    return;
  }
  checkVec(id, `${what}.translation`, actual.translation, expected.translation);
  checkQuat(id, `${what}.rotation`, actual.rotation, expected.rotation);
}

// `emscripten::val` arrays come back as real JS arrays, but a `.length` read on a
// detached handle throws rather than returning undefined, so be explicit.
function checkMatrix(id, actual, expected) {
  if (!actual || actual.length !== 9) {
    fail(id, `matrix: expected 9 components, got ${actual && actual.length}`);
    return;
  }
  for (let i = 0; i < 9; i += 1) {
    checkNumber(id, `matrix[${i}]`, actual[i], expected[i]);
  }
}

function checkStrings(id, what, actual, expected) {
  checked += 1;
  const a = JSON.stringify(Array.from(actual ?? []));
  const e = JSON.stringify(expected);
  if (a !== e) {
    fail(id, `${what}: expected ${e}, got ${a}`);
  }
}

// --- robot trees ---------------------------------------------------------------
//
// Built once per robot and reused. embind does not garbage collect, so every tree
// is deleted at the end; leaking one leaks WASM heap, and the heap is fixed at
// 16MB on purpose (see build.sh).
const trees = new Map();

function treeFor(name) {
  let tree = trees.get(name);
  if (tree === undefined) {
    tree = new geom.RobotTree(golden.robots[name]);
    trees.set(name, tree);
  }
  return tree;
}

// --- dispatch -------------------------------------------------------------------

const IDENTITY = { translation: { x: 0, y: 0, z: 0 }, rotation: { w: 1, x: 0, y: 0, z: 0 } };

function run(testCase) {
  const { input: i } = testCase;
  switch (testCase.op) {
    case "quat_normalize":
      return { quaternion: geom.quatNormalize(i.q) };
    case "quat_conjugate":
      return { quaternion: geom.quatConjugate(i.q) };
    case "quat_inverse":
      return { quaternion: geom.quatInverse(i.q) };
    case "quat_multiply":
      return { quaternion: geom.quatMultiply(i.a, i.b) };
    case "quat_rotate":
      return { vector: geom.quatRotate(i.q, i.v) };
    case "quat_from_axis_angle":
      return { quaternion: geom.quatFromAxisAngle(i.axis, i.angle) };
    case "quat_to_axis_angle":
      return geom.quatToAxisAngle(i.q);
    case "quat_from_rpy":
      return { quaternion: geom.quatFromRpy(i.roll, i.pitch, i.yaw) };
    case "quat_to_rpy":
      return geom.quatToRpy(i.q);
    case "quat_from_yaw":
      return { quaternion: geom.quatFromYaw(i.yaw) };
    case "quat_to_yaw":
      return { yaw: geom.quatToYaw(i.q) };
    case "quat_to_matrix":
      return { matrix: geom.quatToMatrix(i.q) };
    case "quat_from_matrix":
      // The golden file carries no tolerance for this op, so pass the core's own
      // default (quaternion.hpp) rather than inventing a looser one here.
      return { quaternion: geom.quatFromMatrix(i.matrix, 1e-6) };
    case "quat_slerp":
      return { quaternion: geom.quatSlerp(i.a, i.b, i.t) };
    case "quat_nlerp":
      return { quaternion: geom.quatNlerp(i.a, i.b, i.t) };
    case "transform_normalize":
      return { transform: geom.transformNormalize(i.t) };
    case "transform_compose":
      return { transform: geom.transformCompose(i.parent, i.child) };
    case "transform_inverse":
      return { transform: geom.transformInverse(i.t) };
    case "transform_apply":
      return { vector: geom.transformApply(i.t, i.point) };
    case "transform_relative":
      return { transform: geom.transformRelative(i.parent, i.child) };
    // --- geodetic (CONVENTIONS.md section 9) ---
    case "geo_metres_per_degree": {
      const scale = geom.geoMetresPerDegree(i.latitude_deg);
      return {
        metres_per_degree_latitude: scale.latitude,
        metres_per_degree_longitude: scale.longitude,
      };
    }
    case "geo_normalize_longitude_deg":
      return { longitude_deg: geom.geoNormalizeLongitudeDeg(i.longitude_deg) };
    case "geo_enu_between":
      return { vector: geom.geoEnuBetween(i.origin, i.target) };
    case "geo_offset":
      return { geodetic: geom.geoOffset(i.origin, i.enu) };
    case "geo_ground_distance":
      return { distance: geom.geoGroundDistance(i.a, i.b) };
    case "geo_to_local":
      return { vector: geom.geoToLocal(i.anchor, i.position) };
    case "geo_from_local":
      return { geodetic: geom.geoFromLocal(i.anchor, i.position) };
    case "geo_to_local_pose":
      return { transform: geom.geoToLocalPose(i.anchor, i.pose) };
    case "geo_from_local_pose": {
      // The JS surface hands back one flat GeoPose, the way a caller wants it;
      // the golden file names the two halves separately.
      const pose = geom.geoFromLocalPose(i.anchor, i.pose);
      return { geodetic: pose, quaternion: pose.rotation };
    }
    case "geo_quat_from_compass_heading":
      return { quaternion: geom.geoQuatFromCompassHeading(i.heading_deg) };
    case "geo_to_compass_heading_deg":
      return { heading_deg: geom.geoToCompassHeadingDeg(i.q) };
    case "geo_quat_from_ned_rpy":
      return { quaternion: geom.geoQuatFromNedRpy(i.roll, i.pitch, i.yaw) };
    case "geo_to_ned_rpy":
      return geom.geoToNedRpy(i.q);
    case "joint_type_from_string":
      return { joint_type: geom.jointTypeFromString(i.text) };
    case "fk_frame_pose":
      return treeFor(i.robot).framePose(i.frame, i.base ?? IDENTITY, i.joint_positions);
    default:
      throw new Error(`unhandled op ${testCase.op}`);
  }
}

function checkOk(testCase, actual) {
  const { id, expect: e } = testCase;
  if (e.quaternion !== undefined) checkQuat(id, "quaternion", actual.quaternion, e.quaternion);
  if (e.vector !== undefined) checkVec(id, "vector", actual.vector, e.vector);
  if (e.transform !== undefined) checkTransform(id, "transform", actual.transform, e.transform);
  if (e.matrix !== undefined) checkMatrix(id, actual.matrix, e.matrix);
  if (e.axis !== undefined) checkVec(id, "axis", actual.axis, e.axis);
  if (e.angle !== undefined) checkNumber(id, "angle", actual.angle, e.angle);
  if (e.roll !== undefined) checkNumber(id, "roll", actual.roll, e.roll);
  if (e.pitch !== undefined) checkNumber(id, "pitch", actual.pitch, e.pitch);
  if (e.yaw !== undefined) checkNumber(id, "yaw", actual.yaw, e.yaw);
  if (e.geodetic !== undefined) checkGeodetic(id, "geodetic", actual.geodetic, e.geodetic);
  if (e.distance !== undefined) checkNumber(id, "distance", actual.distance, e.distance);
  if (e.heading_deg !== undefined) checkNumber(id, "heading_deg", actual.heading_deg, e.heading_deg);
  if (e.longitude_deg !== undefined) {
    checkNumber(id, "longitude_deg", actual.longitude_deg, e.longitude_deg);
  }
  if (e.metres_per_degree_latitude !== undefined) {
    checkNumber(
      id,
      "metres_per_degree_latitude",
      actual.metres_per_degree_latitude,
      e.metres_per_degree_latitude,
    );
    checkNumber(
      id,
      "metres_per_degree_longitude",
      actual.metres_per_degree_longitude,
      e.metres_per_degree_longitude,
    );
  }
  if (e.joint_type !== undefined) {
    checked += 1;
    if (actual.joint_type !== e.joint_type) {
      fail(id, `joint_type: expected ${e.joint_type}, got ${actual.joint_type}`);
    }
  }
  if (e.available !== undefined) {
    checked += 1;
    if (actual.available !== e.available) {
      fail(id, `available: expected ${e.available}, got ${actual.available}`);
    }
    if (e.available === false && e.missing_joints !== undefined) {
      checkStrings(id, "missingJoints", actual.missingJoints, e.missing_joints);
    }
  }
}

// A structural failure crosses as a thrown `{code, subject, detail}` (embind.cpp),
// never as a status field -- so an expected-error case that does NOT throw is a
// failure, and vice versa.
function runCase(testCase) {
  const { id, expect: e } = testCase;

  if (testCase.op === "fk_tree") {
    const tree = treeFor(testCase.input.robot);
    checked += 1;
    if (tree.rootLink() !== e.root_link) fail(id, `root_link: got ${tree.rootLink()}`);
    checked += 1;
    if (tree.baseFrame() !== e.base_frame) fail(id, `base_frame: got ${tree.baseFrame()}`);
    checked += 1;
    if (tree.isUsable() !== e.usable) fail(id, `usable: got ${tree.isUsable()}`);
    checked += 1;
    if (tree.isArticulated() !== e.articulated) fail(id, `articulated: got ${tree.isArticulated()}`);
    checkStrings(id, "frames", tree.frames(), e.frames);
    // Code AND subject: comparing codes alone would pass a tree that blamed the
    // wrong joint, which is most of what these structural errors are for.
    checkStrings(
      id,
      "errors",
      Array.from(tree.errors()).map((err) => `${err.code}/${err.subject}`),
      e.errors.map((err) => `${err.code}/${err.subject}`),
    );
    return;
  }

  const expectsError = e.status !== undefined && e.status !== "ok";
  let actual;
  try {
    actual = run(testCase);
  } catch (err) {
    if (!expectsError) {
      fail(id, `unexpected error ${err && (err.code ?? err.message ?? err)}`);
      return;
    }
    checked += 1;
    if (err.code !== e.status) fail(id, `error code: expected ${e.status}, got ${err.code}`);
    if (e.error_subject !== undefined) {
      checked += 1;
      if (err.subject !== e.error_subject) {
        fail(id, `error subject: expected ${e.error_subject}, got ${err.subject}`);
      }
    }
    return;
  }

  if (expectsError) {
    fail(id, `expected error ${e.status}, but the call succeeded`);
    return;
  }
  checkOk(testCase, actual);
}

// --- go -------------------------------------------------------------------------

const coreVersion = geom.coreVersion();
for (const testCase of golden.cases) {
  runCase(testCase);
}
for (const tree of trees.values()) {
  tree.delete();
}

const label = `core ${coreVersion}, golden ${golden.version}`;
if (failures.length > 0) {
  console.error(`FAIL (${label}): ${failures.length} of ${golden.cases.length} cases`);
  for (const line of failures.slice(0, 40)) console.error(`  ${line}`);
  if (failures.length > 40) console.error(`  ... and ${failures.length - 40} more`);
  process.exit(1);
}
console.log(`ok (${label}): ${golden.cases.length} cases, ${checked} assertions, tolerance ${TOL}`);
