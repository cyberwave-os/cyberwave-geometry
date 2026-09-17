"""Per-call cost of going through the C ABI, for the unbatched-is-fine claim.

Not a pytest: a timing assertion in CI is a flake generator. Run it by hand
when changing the binding or arguing about whether a call site can afford the
shared core.

    python tests/benchmark.py
"""

from __future__ import annotations

import time

from cyberwave_geometry import JointType, Quaternion, Transform, Vector3
from cyberwave_geometry import quaternion as quat
from cyberwave_geometry import transform as tf
from cyberwave_geometry.fk import (
    JointDescription,
    RobotDescription,
    RobotTree,
    SensorDescription,
)


def bench(label: str, call, iterations: int) -> float:
    call()  # warm the ctypes call path
    start = time.perf_counter()
    for _ in range(iterations):
        call()
    elapsed = time.perf_counter() - start
    per_call_us = elapsed / iterations * 1e6
    print(
        f"  {label:<44s} {per_call_us:8.3f} us/call   {iterations / elapsed:12,.0f} /s"
    )
    return per_call_us


def arm(joint_count: int) -> RobotDescription:
    links = ["base_link"] + [f"link_{index}" for index in range(joint_count)]
    joints = [
        JointDescription(
            name=f"joint_{index}",
            parent_link=links[index],
            child_link=links[index + 1],
            type=JointType.REVOLUTE,
            origin=Transform(Vector3(0.2, 0, 0.1), Quaternion.identity()),
            axis=Vector3(0, 0, 1) if index % 2 else Vector3(0, 1, 0),
        )
        for index in range(joint_count)
    ]
    sensors = [SensorDescription("camera", links[-1], Transform(Vector3(0.05, 0, 0)))]
    return RobotDescription(links=links, joints=joints, sensors=sensors)


def main() -> None:
    q = quat.from_rpy(0.1, 0.2, 0.3)
    other = quat.from_yaw(0.7)
    v = Vector3(1.0, -2.0, 3.0)
    a = Transform(Vector3(1, 2, 3), q)
    b = Transform(Vector3(-1, 0.5, 2), other)

    print("\nscalar ops (the FFI floor)")
    bench("quat.multiply", lambda: quat.multiply(q, other), 200_000)
    bench("quat.normalize", lambda: quat.normalize(q), 200_000)
    bench("quat.rotate_unit", lambda: quat.rotate_unit(q, v), 200_000)
    bench("quat.from_rpy", lambda: quat.from_rpy(0.1, 0.2, 0.3), 200_000)
    bench("transform.compose", lambda: tf.compose(a, b), 200_000)

    print("\nforward kinematics")
    for joint_count in (2, 6, 12):
        description = arm(joint_count)
        tree = RobotTree(description)
        positions = {f"joint_{index}": 0.1 * index for index in range(joint_count)}
        frames = list(tree.frames)
        bench(
            f"RobotTree(...) build, {joint_count} joints",
            lambda d=description: RobotTree(d),
            20_000,
        )
        bench(
            f"frame_pose, tip of {joint_count}-joint arm",
            lambda t=tree, p=positions: t.frame_pose("camera", joint_positions=p),
            50_000,
        )
        per_call = bench(
            f"frame_poses, all {len(frames)} frames",
            lambda t=tree, f=frames, p=positions: t.frame_poses(f, joint_positions=p),
            10_000,
        )
        # Expect this to land on the single-call cost, not below it:
        # frame_poses is a Python-side loop, and the C++ compute_frame_poses
        # batch entry point is not exposed through the C ABI. The numbers below
        # are why that is fine rather than an oversight to fix.
        print(f"  {'':44s} {per_call / len(frames):8.3f} us/frame (one FFI call each)")

    print(
        "\nA telemetry tick recording every frame of a 12-joint arm costs well under\n"
        "a millisecond, so the unbatched core is not the constraint. The tensor path\n"
        "in cyberwave-rl remains the documented exception.\n"
    )


if __name__ == "__main__":
    main()
