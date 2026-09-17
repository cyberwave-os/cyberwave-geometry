"""Batched quaternion and rotation kernels on PyTorch tensors.

The C++ core next door is the authority for *scalar* geometry: one pose, one
frame, one transform. It is deliberately unbatched, and calling it once per
environment from an RL rollout would be a large regression. This module is the
authority for the other case -- the same operations applied to a whole batch of
tensors on a GPU, with autograd.

The two agree numerically. ``tests/test_torch_ops.py`` checks these kernels
against the same ``golden/geometry_golden.json`` the C++ and ctypes bindings are
checked against, one batch entry per golden case, so "the tensor path and the
scalar path compute the same rotation" is a tested claim rather than an
intention.

Conventions (see ``common/geometry/CONVENTIONS.md``). The Hamilton product,
the fixed-axis XYZ roll/pitch/yaw and the numerics match the scalar core; the
component order does **not**:

* quaternions are **wxyz** in the **last** dimension, ``(..., 4)`` -- unlike the
  scalar core's ``xyzw``, because every consumer of this module is a simulator
  surface (IsaacLab, mjlab, MuJoCo) where ``wxyz`` is not ours to choose. Use
  ``Quaternion.to_wxyz()`` to feed a scalar quaternion into a batch;
* vectors are ``(..., 3)``;
* rotation matrices are ``(..., 3, 3)``;
* products are Hamilton: ``quat_mul(a, b)`` applies ``b`` first, then ``a``;
* roll/pitch/yaw is fixed-axis XYZ, the URDF convention.

Everything here preserves the device and dtype of its inputs, broadcasts over
leading dimensions, and is differentiable.

**Torch is an optional dependency.** This module is not imported by
``cyberwave_geometry/__init__.py``; importing the scalar geometry must not
require torch. Install it with the extra::

    pip install "cyberwave-geometry[torch]"

**TorchScript.** Each kernel is written once, as a plain eager function, and
exposed as a scripted callable. Set ``CYBERWAVE_GEOMETRY_TORCH_JIT=0`` to get
the eager functions instead -- mjlab's scripted kernels are unusable in some
CUDA environments (their fusion needs an nvrtc builtins library that is not
always present), and that is exactly the situation the demos hand-rolled these
formulas to escape. Same source either way; only the wrapper changes.
"""

from __future__ import annotations

import math
import os
import warnings
from collections.abc import Callable
from typing import TypeVar

try:
    import torch
    from torch import Tensor
except ImportError as _error:  # pragma: no cover - depends on the install
    # Without this the failure surfaces as "cannot import name 'torch' from
    # 'cyberwave_geometry'", which reads like a typo rather than a missing
    # optional dependency.
    raise ImportError(
        "cyberwave_geometry.torch requires PyTorch, which is an optional "
        "dependency of cyberwave-geometry. Install it with:\n"
        '    pip install "cyberwave-geometry[torch]"\n'
        "The scalar geometry in cyberwave_geometry does not need it."
    ) from _error

__all__ = [
    "direction_to_quat",
    "jit_enabled",
    "normalize",
    "quat_apply",
    "quat_conjugate",
    "quat_from_matrix",
    "quat_from_z_direction",
    "quat_mul",
    "quat_normalize",
    "quat_rotate",
    "quat_to_euler_xyz",
    "quat_to_matrix",
    "quat_unique",
    "sample_direction_in_cone",
]

#: Below this a vector or quaternion has no usable direction. Matches the
#: scalar core's kMinQuaternionNorm so the two agree on what is degenerate.
#:
#: TorchScript cannot read a module-level global from inside a compiled
#: function -- "Perhaps it is a closed over global variable?". Default argument
#: values *are* baked in at definition time, so every kernel that needs this
#: takes it as ``eps: float = _EPS`` and uses the parameter. That keeps one
#: source for the number without scattering the literal.
_EPS: float = 1e-9


def _jit_requested() -> bool:
    return os.environ.get("CYBERWAVE_GEOMETRY_TORCH_JIT", "1").strip().lower() not in (
        "0",
        "false",
        "no",
    )


_T = TypeVar("_T", bound=Callable[..., object])


def _script(function: _T) -> _T:
    """Return the TorchScript-compiled function, or the original if unavailable.

    Compilation failing is worth a warning rather than silence: the eager
    fallback is numerically identical but slower, and a caller who asked for
    scripted kernels should learn they did not get them.
    """
    if not _jit_requested():
        return function
    try:
        return torch.jit.script(function)  # type: ignore[return-value]
    except Exception as error:  # pragma: no cover - depends on the torch build
        warnings.warn(
            f"cyberwave_geometry.torch: could not TorchScript-compile "
            f"{function.__name__} ({error}); using the eager implementation. "
            f"Set CYBERWAVE_GEOMETRY_TORCH_JIT=0 to silence this.",
            RuntimeWarning,
            stacklevel=2,
        )
        return function


def jit_enabled() -> bool:
    """Whether the exported callables are TorchScript-compiled."""
    return isinstance(quat_mul, torch.jit.ScriptFunction)


# ---------------------------------------------------------------------------
# Implementations. One formula each; the scripted callables below wrap these.
# ---------------------------------------------------------------------------


def _normalize(x: Tensor, eps: float = _EPS) -> Tensor:
    return x / torch.clamp(torch.linalg.vector_norm(x, dim=-1, keepdim=True), min=eps)


def _quat_normalize(q: Tensor) -> Tensor:
    return _normalize(q)


def _quat_conjugate(q: Tensor) -> Tensor:
    return torch.cat([q[..., 0:1], -q[..., 1:4]], dim=-1)


def _quat_unique(q: Tensor) -> Tensor:
    # q and -q are the same rotation; pick w >= 0 so equal rotations compare
    # equal and a learned policy sees one representation, not two.
    return torch.where(q[..., 0:1] < 0.0, -q, q)


def _quat_mul(a: Tensor, b: Tensor) -> Tensor:
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return torch.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dim=-1,
    )


def _quat_rotate(quat: Tensor, vec: Tensor) -> Tensor:
    # v + 2w(u x v) + 2u x (u x v), with u the vector part. Leading dimensions
    # are aligned by inserting point axes into the quaternion, so a (B, 4)
    # rotation applies to (B, P, 3) points -- the case mjlab's quat_apply
    # cannot express and the reason cyberwave-rl grew its own copy.
    #
    # The guards came with it. Without them an xyzw quaternion, or a tensor
    # whose components are not last, reaches torch.linalg.cross and fails
    # there -- an error naming neither the expected layout nor this function.
    assert quat.shape[-1] == 4, "quat must have last dim 4 (w, x, y, z)"
    assert vec.shape[-1] == 3, "vec must have last dim 3 (x, y, z)"
    w = quat[..., 0:1]
    xyz = quat[..., 1:4]
    while xyz.dim() < vec.dim():
        xyz = xyz.unsqueeze(-2)
        w = w.unsqueeze(-2)
    t = 2.0 * torch.linalg.cross(xyz, vec, dim=-1)
    return vec + w * t + torch.linalg.cross(xyz, t, dim=-1)


def _quat_to_matrix(q: Tensor) -> Tensor:
    q = _quat_normalize(q)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    row0 = torch.stack(
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        dim=-1,
    )
    row1 = torch.stack(
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        dim=-1,
    )
    row2 = torch.stack(
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        dim=-1,
    )
    return torch.stack([row0, row1, row2], dim=-2)


def _safe_matrix_divide(
    numerator: Tensor, denominator: Tensor, eps: float = _EPS
) -> Tensor:
    return numerator / torch.clamp(denominator, min=eps)


def _quat_from_matrix(m: Tensor) -> Tensor:
    # Use Shepperd's largest-diagonal construction. Taking magnitudes from the
    # diagonal and signs from the skew part looks pleasantly branch-free, but
    # it fails for every exact half-turn: a 180-degree rotation matrix is
    # symmetric, so all of its skew terms are zero and the result can collapse
    # to a zero quaternion (or lose the signs of an arbitrary half-turn axis).
    # Compute all four numerically stable candidates and select one per lane;
    # torch.where keeps this batched and TorchScript-compatible.
    m00, m11, m22 = m[..., 0, 0], m[..., 1, 1], m[..., 2, 2]
    trace = 1.0 + m00 + m11 + m22

    trace_s = 2.0 * torch.sqrt(torch.clamp(trace, min=0.0))
    trace_candidate = torch.stack(
        [
            0.25 * trace_s,
            _safe_matrix_divide(m[..., 2, 1] - m[..., 1, 2], trace_s),
            _safe_matrix_divide(m[..., 0, 2] - m[..., 2, 0], trace_s),
            _safe_matrix_divide(m[..., 1, 0] - m[..., 0, 1], trace_s),
        ],
        dim=-1,
    )

    x_s = 2.0 * torch.sqrt(torch.clamp(1.0 + m00 - m11 - m22, min=0.0))
    x_candidate = torch.stack(
        [
            _safe_matrix_divide(m[..., 2, 1] - m[..., 1, 2], x_s),
            0.25 * x_s,
            _safe_matrix_divide(m[..., 0, 1] + m[..., 1, 0], x_s),
            _safe_matrix_divide(m[..., 0, 2] + m[..., 2, 0], x_s),
        ],
        dim=-1,
    )

    y_s = 2.0 * torch.sqrt(torch.clamp(1.0 - m00 + m11 - m22, min=0.0))
    y_candidate = torch.stack(
        [
            _safe_matrix_divide(m[..., 0, 2] - m[..., 2, 0], y_s),
            _safe_matrix_divide(m[..., 0, 1] + m[..., 1, 0], y_s),
            0.25 * y_s,
            _safe_matrix_divide(m[..., 1, 2] + m[..., 2, 1], y_s),
        ],
        dim=-1,
    )

    z_s = 2.0 * torch.sqrt(torch.clamp(1.0 - m00 - m11 + m22, min=0.0))
    z_candidate = torch.stack(
        [
            _safe_matrix_divide(m[..., 1, 0] - m[..., 0, 1], z_s),
            _safe_matrix_divide(m[..., 0, 2] + m[..., 2, 0], z_s),
            _safe_matrix_divide(m[..., 1, 2] + m[..., 2, 1], z_s),
            0.25 * z_s,
        ],
        dim=-1,
    )

    x_largest = (m00 >= m11) & (m00 >= m22)
    y_largest = (~x_largest) & (m11 >= m22)
    candidate = torch.where(x_largest.unsqueeze(-1), x_candidate, z_candidate)
    candidate = torch.where(y_largest.unsqueeze(-1), y_candidate, candidate)
    # A mathematically exact half-turn has trace zero. Round-off in constructing
    # a symmetric matrix can make it a tiny positive number, in which case the
    # trace candidate divides by a near-zero skew term and loses the axis. The
    # diagonal candidates are the stable choice in that small neighbourhood too.
    candidate = torch.where((trace > 1e-6).unsqueeze(-1), trace_candidate, candidate)
    return _quat_normalize(torch.nan_to_num(candidate, nan=0.0))


def _quat_to_euler_xyz(q: Tensor) -> Tensor:
    """Fixed-axis XYZ roll/pitch/yaw, matching the scalar core exactly.

    Including at gimbal lock, which is a deliberate departure from mjlab's
    ``euler_xyz_from_quat`` and from the copies the Hitachi tasks grew. At
    pitch = +/- pi/2 the plain atan2 pair does not merely pick an arbitrary
    split of a degenerate degree of freedom -- it returns a pair that
    reconstructs a *different rotation*. For ``from_rpy(0.3, pi/2, 1.2)`` it
    yields roll 0.245 / yaw 0.588, a difference of 0.343 where the input's was
    0.900, and rebuilding from it gives the wrong quaternion.

    So the pole is handled the way the scalar core handles it: attribute the
    whole rotation to yaw, report roll as zero, which does reconstruct. The two
    paths then agree everywhere, which is the point of one shared module.
    Outside the pole this is the same atan2 pair as before, so ordinary
    attitudes are untouched -- and the pole is decided on cos(pitch), so "near
    the pole" means near it, not merely too close to tell apart in sin(pitch)
    (see ``kMinCosPitch`` in the scalar core). How near is dtype-dependent; the
    threshold below says why.
    """
    # Normalized first, exactly as _quat_to_matrix is, and for the same reason:
    # the scalar quat::to_rpy opens with normalize(), so skipping it here is a
    # silent divergence rather than an optimization. The formula below is not
    # scale-invariant -- sin_pitch scales as |q|^2 while the 1.0 in each atan2
    # denominator does not, so a non-unit input both shifts the angles and, once
    # |q|^2 * |sin_pitch| reaches 1, trips `locked` into reporting a gimbal lock
    # that is not there.
    q = _quat_normalize(q)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    sin_pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0)
    # First column of the rotation matrix: |cos(pitch)| is its length, yaw its
    # direction.
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r10 = 2.0 * (w * z + x * y)
    cos_pitch = torch.hypot(r00, r10)

    # The pole is decided on cos(pitch), whose noise floor is a few ulp of 1 and
    # so depends on the dtype -- unlike the flat 1e-12 the scalar core can afford
    # in double. At float32 the at-pole hypot is ~5e-8, five hundred times the
    # scalar threshold, so a fixed 1e-12 would never fire and the kernel would
    # read a real roll and yaw out of rounding noise: pi and pi for a rotation
    # whose roll is zero. Hence max(scalar threshold, 8 ulp), which leaves
    # float64 on exactly the core's 1e-12 and widens only where the precision to
    # resolve the pole does not exist.
    one = torch.ones((), dtype=q.dtype, device=q.device)
    ulp = torch.nextafter(one, one + one) - one
    locked = cos_pitch < torch.clamp(8.0 * ulp, min=1e-12)

    roll = torch.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    yaw = torch.atan2(r10, r00)

    # atan2 of the pair rather than asin(sin_pitch): asin has infinite slope at
    # 1, so a sin_pitch 2e-16 short of it comes back 2e-8 short of pi/2, and
    # rebuilding from that misses the original quaternion by ~7e-9. The scalar
    # core does the same, which is what lets the two agree to 1e-12 here.
    pitch = torch.atan2(sin_pitch, cos_pitch)
    roll = torch.where(locked, torch.zeros_like(roll), roll)
    yaw = torch.where(locked, 2.0 * torch.atan2(z, w), yaw)
    return torch.stack([roll, pitch, yaw], dim=-1)


def _quat_from_z_direction(direction: Tensor, eps: float = _EPS) -> Tensor:
    # Shortest rotation taking +Z onto `direction`: zero roll about the result.
    # The two degenerate cases are handled explicitly rather than by an epsilon
    # in a denominator -- antiparallel has no shortest arc, and any half turn
    # perpendicular to Z is as good as another, so +X is chosen.
    direction = _normalize(direction, eps)
    z_axis = torch.zeros_like(direction)
    z_axis[..., 2] = 1.0

    dot = torch.sum(z_axis * direction, dim=-1).clamp(-1.0, 1.0)
    cross = torch.linalg.cross(z_axis, direction, dim=-1)

    w = torch.sqrt(torch.clamp((1.0 + dot) / 2.0, min=0.0))
    xyz = cross / torch.clamp(2.0 * w.unsqueeze(-1), min=eps)
    q = torch.cat([w.unsqueeze(-1), xyz], dim=-1)

    identity = torch.zeros_like(q)
    identity[..., 0] = 1.0
    half_turn = torch.zeros_like(q)
    half_turn[..., 1] = 1.0

    q = torch.where((dot > 0.9999).unsqueeze(-1), identity, q)
    q = torch.where((dot < -0.9999).unsqueeze(-1), half_turn, q)
    return _quat_unique(_quat_normalize(q))


def _direction_to_quat(direction: Tensor, eps: float = _EPS) -> Tensor:
    # A full orthonormal frame with +Z on `direction`, built by Gram-Schmidt.
    # Distinct from quat_from_z_direction: the roll about the direction is
    # whatever the Gram-Schmidt reference produces, not minimal. Kept because
    # sample_direction_in_cone only needs *a* frame, and changing which one
    # would change the sampled directions for a given random draw.
    z = _normalize(direction, eps)
    reference = torch.zeros_like(z)
    reference[..., 2] = 1.0
    alternate = torch.zeros_like(z)
    alternate[..., 0] = 1.0
    up = torch.where((z[..., 2:3].abs() < 0.9), reference, alternate)

    x = _normalize(up - torch.sum(up * z, dim=-1, keepdim=True) * z, eps)
    y = torch.linalg.cross(z, x, dim=-1)
    matrix = torch.stack([x, y, z], dim=-1)
    return _quat_from_matrix(matrix)


def _sample_direction_in_cone(axis: Tensor, cone_angle_rad: float, n: int) -> Tensor:
    # Uniform over the spherical cap, not over the angle: cos(theta) uniform is
    # what makes the density constant per unit solid angle.
    cos_theta_max = math.cos(cone_angle_rad)
    cos_theta = (
        torch.rand(n, device=axis.device, dtype=axis.dtype) * (1.0 - cos_theta_max)
        + cos_theta_max
    )
    sin_theta = torch.sqrt(torch.clamp(1.0 - cos_theta * cos_theta, min=0.0))
    phi = torch.rand(n, device=axis.device, dtype=axis.dtype) * (2.0 * math.pi)
    local = torch.stack(
        [sin_theta * torch.cos(phi), sin_theta * torch.sin(phi), cos_theta], dim=-1
    )
    return _quat_rotate(_direction_to_quat(axis), local)


# ---------------------------------------------------------------------------
# Exported callables
# ---------------------------------------------------------------------------

normalize = _script(_normalize)
quat_normalize = _script(_quat_normalize)
quat_conjugate = _script(_quat_conjugate)
quat_unique = _script(_quat_unique)
quat_mul = _script(_quat_mul)
quat_rotate = _script(_quat_rotate)
#: mjlab's name for the same operation. Aliased rather than reimplemented.
quat_apply = quat_rotate
quat_to_matrix = _script(_quat_to_matrix)
quat_from_matrix = _script(_quat_from_matrix)
quat_to_euler_xyz = _script(_quat_to_euler_xyz)
quat_from_z_direction = _script(_quat_from_z_direction)
direction_to_quat = _script(_direction_to_quat)
sample_direction_in_cone = _script(_sample_direction_in_cone)
