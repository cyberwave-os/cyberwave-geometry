"""The batched torch kernels agree with the scalar C++ core.

``cyberwave_geometry.torch`` exists because the scalar core cannot be called
once per environment in an RL rollout. That is a performance argument, not a
licence to compute something different: these tests replay the same
``golden/geometry_golden.json`` the C++ suite and the ctypes binding are checked
against, one batch entry per golden case, so the two paths are held to one
answer.

Where they legitimately differ -- gimbal lock, and the strict-versus-clamped
handling of degenerate input -- the difference is asserted explicitly rather
than skipped.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

torch = pytest.importorskip("torch", reason="cyberwave_geometry.torch needs torch")

from cyberwave_geometry import torch as cwt  # noqa: E402

GOLDEN_PATH = Path(__file__).resolve().parents[3] / "golden" / "geometry_golden.json"
DEVICES = ["cpu"] + (["cuda"] if torch.cuda.is_available() else [])
DTYPES = [torch.float32, torch.float64]

#: float64 matches the core to well below the golden tolerance; float32 carries
#: about seven digits, so it gets a tolerance that reflects that rather than a
#: pretence of agreement it cannot deliver.
TOLERANCE = {torch.float64: 1e-12, torch.float32: 2e-6}


@pytest.fixture(scope="module")
def golden() -> dict:
    assert GOLDEN_PATH.exists(), f"{GOLDEN_PATH} is missing"
    return json.loads(GOLDEN_PATH.read_text())


def _quat(data: dict) -> list[float]:
    return [data["w"], data["x"], data["y"], data["z"]]


def _vec(data: dict) -> list[float]:
    return [data["x"], data["y"], data["z"]]


def _cases(golden: dict, op: str) -> list[dict]:
    """Golden cases for one op that the torch kernels are expected to match.

    Only ``status: ok`` ones. A degenerate input is an *error* in the strict
    scalar core and a clamped value in the tensor kernels -- a batch cannot
    raise for one lane -- so those cases are covered separately below.
    """
    return [
        c for c in golden["cases"] if c["op"] == op and c["expect"]["status"] == "ok"
    ]


# --- golden parity, batched -------------------------------------------------


@pytest.mark.parametrize("dtype", DTYPES)
@pytest.mark.parametrize("device", DEVICES)
def test_quat_mul_matches_the_scalar_core(golden, device, dtype):
    cases = _cases(golden, "quat_multiply")
    assert cases, "no quat_multiply cases in the golden file"
    a = torch.tensor(
        [_quat(c["input"]["a"]) for c in cases], device=device, dtype=dtype
    )
    b = torch.tensor(
        [_quat(c["input"]["b"]) for c in cases], device=device, dtype=dtype
    )
    expected = torch.tensor(
        [_quat(c["expect"]["quaternion"]) for c in cases], device=device, dtype=dtype
    )
    assert torch.allclose(cwt.quat_mul(a, b), expected, atol=TOLERANCE[dtype])


@pytest.mark.parametrize("dtype", DTYPES)
@pytest.mark.parametrize("device", DEVICES)
def test_quat_conjugate_matches_the_scalar_core(golden, device, dtype):
    cases = _cases(golden, "quat_conjugate")
    q = torch.tensor(
        [_quat(c["input"]["q"]) for c in cases], device=device, dtype=dtype
    )
    expected = torch.tensor(
        [_quat(c["expect"]["quaternion"]) for c in cases], device=device, dtype=dtype
    )
    assert torch.allclose(cwt.quat_conjugate(q), expected, atol=TOLERANCE[dtype])


@pytest.mark.parametrize("dtype", DTYPES)
@pytest.mark.parametrize("device", DEVICES)
def test_quat_normalize_matches_the_scalar_core(golden, device, dtype):
    cases = _cases(golden, "quat_normalize")
    q = torch.tensor(
        [_quat(c["input"]["q"]) for c in cases], device=device, dtype=dtype
    )
    expected = torch.tensor(
        [_quat(c["expect"]["quaternion"]) for c in cases], device=device, dtype=dtype
    )
    assert torch.allclose(cwt.quat_normalize(q), expected, atol=TOLERANCE[dtype])


@pytest.mark.parametrize("dtype", DTYPES)
@pytest.mark.parametrize("device", DEVICES)
def test_quat_rotate_matches_the_scalar_core(golden, device, dtype):
    cases = _cases(golden, "quat_rotate")
    # The golden op normalizes first; the tensor kernel assumes a unit
    # quaternion, so the normalize is done here rather than inside the hot loop.
    q = cwt.quat_normalize(
        torch.tensor(
            [_quat(c["input"]["q"]) for c in cases], device=device, dtype=dtype
        )
    )
    v = torch.tensor([_vec(c["input"]["v"]) for c in cases], device=device, dtype=dtype)
    expected = torch.tensor(
        [_vec(c["expect"]["vector"]) for c in cases], device=device, dtype=dtype
    )
    assert torch.allclose(cwt.quat_rotate(q, v), expected, atol=TOLERANCE[dtype])


@pytest.mark.parametrize("dtype", DTYPES)
@pytest.mark.parametrize("device", DEVICES)
def test_quat_to_matrix_matches_the_scalar_core(golden, device, dtype):
    cases = _cases(golden, "quat_to_matrix")
    q = torch.tensor(
        [_quat(c["input"]["q"]) for c in cases], device=device, dtype=dtype
    )
    expected = torch.tensor(
        [c["expect"]["matrix"] for c in cases], device=device, dtype=dtype
    ).reshape(-1, 3, 3)
    assert torch.allclose(cwt.quat_to_matrix(q), expected, atol=TOLERANCE[dtype])


@pytest.mark.parametrize("dtype", DTYPES)
@pytest.mark.parametrize("device", DEVICES)
def test_quat_from_matrix_matches_the_scalar_core(golden, device, dtype):
    cases = _cases(golden, "quat_from_matrix")
    m = torch.tensor(
        [c["input"]["matrix"] for c in cases], device=device, dtype=dtype
    ).reshape(-1, 3, 3)
    expected = torch.tensor(
        [_quat(c["expect"]["quaternion"]) for c in cases], device=device, dtype=dtype
    )
    # q and -q are the same rotation and the two implementations pick the sign
    # by different rules, so compare canonicalized.
    assert torch.allclose(
        cwt.quat_unique(cwt.quat_from_matrix(m)),
        cwt.quat_unique(expected),
        atol=TOLERANCE[dtype],
    )


@pytest.mark.parametrize("axis", [1, 2, 3])
def test_quat_from_matrix_preserves_exact_half_turns(axis):
    """A zero skew term must not erase an exact 180-degree rotation."""
    q = torch.zeros((1, 4), dtype=torch.float64)
    q[0, axis] = 1.0
    matrix = cwt.quat_to_matrix(q)
    recovered = cwt.quat_from_matrix(matrix)

    assert torch.isfinite(recovered).all()
    assert torch.allclose(cwt.quat_to_matrix(recovered), matrix, atol=1e-12)
    assert torch.allclose(recovered.abs(), q.abs(), atol=1e-12)


@pytest.mark.parametrize("device", DEVICES)
def test_quat_from_matrix_handles_exact_half_turns(device):
    """Symmetric 180-degree matrices must keep their axis and remain unit."""
    axes = torch.tensor(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 2.0, 3.0]],
        device=device,
        dtype=torch.float64,
    )
    axes = axes / axes.norm(dim=-1, keepdim=True)
    identity = torch.eye(3, device=device, dtype=torch.float64).expand(4, 3, 3)
    matrices = 2.0 * axes.unsqueeze(-1) * axes.unsqueeze(-2) - identity

    recovered = cwt.quat_from_matrix(matrices)
    assert torch.allclose(
        recovered.norm(dim=-1),
        torch.ones(4, device=device, dtype=torch.float64),
        atol=1e-12,
    )
    assert torch.allclose(cwt.quat_to_matrix(recovered), matrices, atol=1e-12)


@pytest.mark.parametrize("dtype", DTYPES)
@pytest.mark.parametrize("device", DEVICES)
def test_quat_to_euler_matches_the_scalar_core(golden, device, dtype):
    """Every case, gimbal lock included -- the two paths agree everywhere."""
    cases = _cases(golden, "quat_to_rpy")
    assert cases, "no quat_to_rpy cases in the golden file"
    # Handed to the kernel raw, the golden file's non-unit case included:
    # normalizing here would hide whether the kernel does it, which is the claim
    # under test. The scalar core normalizes, so the batched path has to too.
    q = torch.tensor(
        [_quat(c["input"]["q"]) for c in cases], device=device, dtype=dtype
    )
    expected = torch.tensor(
        [
            [c["expect"]["roll"], c["expect"]["pitch"], c["expect"]["yaw"]]
            for c in cases
        ],
        device=device,
        dtype=dtype,
    )
    assert torch.allclose(cwt.quat_to_euler_xyz(q), expected, atol=TOLERANCE[dtype])


def test_quat_to_euler_keeps_the_roll_merely_near_the_pole():
    """float64 only, and that is the point.

    The batched kernel decides the singularity on cos(pitch) exactly as the
    scalar core does, so 1e-7 rad off vertical it still returns the roll it was
    given. At float32 that roll lives below the epsilon of
    ``1 - 2(y^2 + z^2)``, which is why this case is not in the golden file --
    see the note in ``tests/golden_ops.cpp``.
    """
    from cyberwave_geometry import quaternion as scalar

    for pitch in (math.pi / 2 - 1e-7, -math.pi / 2 + 1e-7):
        original = scalar.from_rpy(0.7, pitch, -1.2)
        expected = scalar.to_rpy(original)
        angles = cwt.quat_to_euler_xyz(
            torch.tensor(
                [[original.w, original.x, original.y, original.z]], dtype=torch.float64
            )
        )[0]
        assert angles[0].item() == pytest.approx(expected.roll, abs=1e-12)
        assert angles[1].item() == pytest.approx(expected.pitch, abs=1e-12)
        assert angles[2].item() == pytest.approx(expected.yaw, abs=1e-12)


def test_quat_to_euler_reconstructs_at_gimbal_lock_where_mjlab_does_not():
    """Why this kernel departs from mjlab's convention at the pole.

    The plain atan2 pair does not merely split a degenerate degree of freedom
    arbitrarily -- it returns angles that rebuild a *different* rotation.
    """
    from cyberwave_geometry import quaternion as scalar

    for pitch in (math.pi / 2, -math.pi / 2):
        original = scalar.from_rpy(0.3, pitch, 1.2)
        angles = cwt.quat_to_euler_xyz(
            torch.tensor(
                [[original.w, original.x, original.y, original.z]], dtype=torch.float64
            )
        )[0]
        assert abs(float(angles[0])) < 1e-9, "roll is attributed to yaw at the pole"

        rebuilt = scalar.from_rpy(float(angles[0]), float(angles[1]), float(angles[2]))
        assert any(
            all(
                abs(sign * getattr(rebuilt, axis) - getattr(original, axis)) < 1e-9
                for axis in ("w", "x", "y", "z")
            )
            for sign in (1.0, -1.0)
        ), "the angles must rebuild the rotation they came from"

        # The formula this replaced does not. Kept as a regression witness so a
        # future "simplification" back to the plain pair fails loudly.
        naive_roll = math.atan2(
            2.0 * (original.w * original.x + original.y * original.z),
            1.0 - 2.0 * (original.x * original.x + original.y * original.y),
        )
        assert abs(naive_roll) > 1e-3, "the naive roll is non-zero here, and wrong"


# --- behaviour the golden file cannot express -------------------------------


@pytest.mark.parametrize("device", DEVICES)
def test_broadcasting_a_pose_over_many_points(device):
    """(B, 4) applied to (B, P, 3) — the case mjlab's quat_apply cannot express
    and the reason cyberwave-rl grew its own copy."""
    quat = cwt.quat_from_z_direction(torch.tensor([[1.0, 0.0, 0.0]], device=device))
    points = torch.randn(1, 7, 3, device=device)
    rotated = cwt.quat_rotate(quat, points)
    assert rotated.shape == points.shape
    # Rotation preserves length, pointwise.
    assert torch.allclose(rotated.norm(dim=-1), points.norm(dim=-1), atol=1e-6)


@pytest.mark.parametrize("device", DEVICES)
def test_leading_dimensions_are_preserved(device):
    quat = cwt.quat_normalize(torch.randn(2, 3, 4, device=device))
    vec = torch.randn(2, 3, 3, device=device)
    assert cwt.quat_rotate(quat, vec).shape == (2, 3, 3)
    assert cwt.quat_mul(quat, quat).shape == (2, 3, 4)
    assert cwt.quat_to_matrix(quat).shape == (2, 3, 3, 3)
    assert cwt.quat_to_euler_xyz(quat).shape == (2, 3, 3)


@pytest.mark.parametrize("dtype", DTYPES)
@pytest.mark.parametrize("device", DEVICES)
def test_device_and_dtype_are_preserved(device, dtype):
    quat = cwt.quat_normalize(torch.randn(4, 4, device=device, dtype=dtype))
    for result in (
        cwt.quat_mul(quat, quat),
        cwt.quat_conjugate(quat),
        cwt.quat_to_matrix(quat),
        cwt.quat_to_euler_xyz(quat),
        cwt.quat_rotate(quat, torch.randn(4, 3, device=device, dtype=dtype)),
        cwt.quat_from_z_direction(torch.randn(4, 3, device=device, dtype=dtype)),
    ):
        assert result.dtype == dtype
        assert result.device.type == device


def test_gradients_flow_through_every_kernel():
    quat = torch.nn.functional.normalize(torch.randn(5, 4, dtype=torch.float64), dim=-1)
    quat.requires_grad_(True)
    vec = torch.randn(5, 3, dtype=torch.float64, requires_grad=True)

    loss = (
        cwt.quat_rotate(quat, vec).sum()
        + cwt.quat_mul(quat, quat).sum()
        + cwt.quat_to_matrix(quat).sum()
        + cwt.quat_to_euler_xyz(quat).sum()
    )
    loss.backward()
    assert quat.grad is not None
    assert torch.isfinite(quat.grad).all()
    assert vec.grad is not None
    assert torch.isfinite(vec.grad).all()


def test_kernels_are_scripted_and_serializable(tmp_path):
    assert cwt.jit_enabled(), "expected TorchScript-compiled kernels by default"
    path = tmp_path / "quat_mul.pt"
    torch.jit.save(cwt.quat_mul, str(path))
    loaded = torch.jit.load(str(path))
    a = cwt.quat_normalize(torch.randn(3, 4, dtype=torch.float32))
    b = cwt.quat_normalize(torch.randn(3, 4, dtype=torch.float32))
    assert torch.allclose(loaded(a, b), cwt.quat_mul(a, b))


# --- edge behaviour ---------------------------------------------------------


def test_direction_alignment_at_both_poles_and_in_between():
    directions = torch.tensor(
        [[0.0, 0.0, 1.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        dtype=torch.float64,
    )
    quat = cwt.quat_from_z_direction(directions)
    # Whatever the rotation is, it must actually take +Z onto the direction.
    z_axis = torch.tensor([0.0, 0.0, 1.0], dtype=torch.float64).expand(4, 3)
    assert torch.allclose(cwt.quat_rotate(quat, z_axis), directions, atol=1e-9)
    assert torch.allclose(
        quat.norm(dim=-1), torch.ones(4, dtype=torch.float64), atol=1e-9
    )
    # Parallel is exactly identity, not merely close.
    assert torch.allclose(
        quat[0], torch.tensor([1.0, 0.0, 0.0, 0.0], dtype=torch.float64)
    )


def test_degenerate_input_clamps_instead_of_raising():
    """The scalar core raises; a batch cannot, so it clamps -- deliberately.

    One bad lane must not take down a rollout of 4096 environments, and there is
    no per-lane exception. The contract is: finite output, unit where a unit
    quaternion is promised.
    """
    zeros = torch.zeros(3, 4, dtype=torch.float64)
    assert torch.isfinite(cwt.quat_normalize(zeros)).all()
    assert torch.isfinite(
        cwt.quat_from_z_direction(torch.zeros(3, 3, dtype=torch.float64))
    ).all()


def test_quat_unique_canonicalizes_the_double_cover():
    quat = torch.tensor(
        [[-0.5, 0.5, 0.5, 0.5], [0.5, -0.5, -0.5, -0.5]], dtype=torch.float64
    )
    unique = cwt.quat_unique(quat)
    assert (unique[..., 0] >= 0).all()
    # Same rotation either way.
    vec = torch.randn(2, 3, dtype=torch.float64)
    assert torch.allclose(
        cwt.quat_rotate(quat, vec), cwt.quat_rotate(unique, vec), atol=1e-12
    )


def test_sampling_stays_inside_the_cone():
    axis = torch.nn.functional.normalize(
        torch.randn(512, 3, dtype=torch.float64), dim=-1
    )
    for half_angle in (0.1, math.pi / 4, math.pi / 2):
        sampled = cwt.sample_direction_in_cone(axis, half_angle, 512)
        assert torch.allclose(
            sampled.norm(dim=-1), torch.ones(512, dtype=torch.float64), atol=1e-9
        )
        cosine = (sampled * axis).sum(dim=-1)
        assert (cosine >= math.cos(half_angle) - 1e-9).all()
