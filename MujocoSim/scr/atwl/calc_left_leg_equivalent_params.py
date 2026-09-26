#!/usr/bin/env python3
"""Calculate left leg equivalent COM distance and pitch inertia from the URDF.

Assumptions used here:
  * left_front1_link ... left_front4_link stay at the URDF default pose.
  * left_rear1_link and left_rear2_link are solved as a 2-DOF serial chain.
  * The wheel joint is moved along its default x-z direction from
    base_link_left_rear1_link_joint, while the requested distance ignores the
    base_link y component.
  * Equivalent inertia is taken about the axis through
    base_link_left_front1_link_joint and parallel to base_link's y axis, using
    the parallel-axis theorem.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple
import xml.etree.ElementTree as ET

import numpy as np


LEG_LINKS = (
    "left_front1_link",
    "left_front2_link",
    "left_front3_link",
    "left_front4_link",
    "left_rear1_link",
    "left_rear2_link",
)

FRONT_JOINT = "base_link_left_front1_link_joint"
REAR_JOINT_1 = "base_link_left_rear1_link_joint"
REAR_JOINT_2 = "left_rear1_link_left_rear2_link_joint"
WHEEL_JOINT = "left_rear2_link_left_wheel_link_joint"


@dataclass
class LinkInertial:
    mass: float
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    inertia: np.ndarray


@dataclass
class Joint:
    name: str
    parent: str
    child: str
    joint_type: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray


def parse_vec(text: str | None, default: str = "0 0 0") -> np.ndarray:
    return np.array([float(v) for v in (text or default).split()], dtype=float)


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = np.linalg.norm(axis)
    if norm == 0.0:
        return np.eye(3)
    x, y, z = axis / norm
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ]
    )


def transform(rotation: np.ndarray | None = None, translation: np.ndarray | None = None) -> np.ndarray:
    tf = np.eye(4)
    if rotation is not None:
        tf[:3, :3] = rotation
    if translation is not None:
        tf[:3, 3] = translation
    return tf


def origin_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    return transform(rpy_matrix(rpy), xyz)


def parse_urdf(urdf_path: Path) -> Tuple[Dict[str, LinkInertial], Dict[str, Joint], Dict[str, List[Joint]]]:
    root = ET.parse(urdf_path).getroot()

    links: Dict[str, LinkInertial] = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue

        origin = inertial.find("origin")
        inertia = inertial.find("inertia")
        if inertia is None:
            raise ValueError(f"Missing inertia element for {link.attrib['name']}")

        links[link.attrib["name"]] = LinkInertial(
            mass=float(inertial.find("mass").attrib["value"]),
            origin_xyz=parse_vec(origin.attrib.get("xyz") if origin is not None else None),
            origin_rpy=parse_vec(origin.attrib.get("rpy") if origin is not None else None),
            inertia=np.array(
                [
                    [
                        float(inertia.attrib["ixx"]),
                        float(inertia.attrib.get("ixy", "0")),
                        float(inertia.attrib.get("ixz", "0")),
                    ],
                    [
                        float(inertia.attrib.get("ixy", "0")),
                        float(inertia.attrib["iyy"]),
                        float(inertia.attrib.get("iyz", "0")),
                    ],
                    [
                        float(inertia.attrib.get("ixz", "0")),
                        float(inertia.attrib.get("iyz", "0")),
                        float(inertia.attrib["izz"]),
                    ],
                ],
                dtype=float,
            ),
        )

    joints: Dict[str, Joint] = {}
    children_by_parent: Dict[str, List[Joint]] = {}
    for joint_el in root.findall("joint"):
        origin = joint_el.find("origin")
        axis_el = joint_el.find("axis")
        joint = Joint(
            name=joint_el.attrib["name"],
            parent=joint_el.find("parent").attrib["link"],
            child=joint_el.find("child").attrib["link"],
            joint_type=joint_el.attrib.get("type", "fixed"),
            origin_xyz=parse_vec(origin.attrib.get("xyz") if origin is not None else None),
            origin_rpy=parse_vec(origin.attrib.get("rpy") if origin is not None else None),
            axis=parse_vec(axis_el.attrib.get("xyz") if axis_el is not None else None, "1 0 0"),
        )
        joints[joint.name] = joint
        children_by_parent.setdefault(joint.parent, []).append(joint)

    return links, joints, children_by_parent


def joint_transform(joint: Joint, angle: float) -> np.ndarray:
    tf = origin_transform(joint.origin_xyz, joint.origin_rpy)
    if joint.joint_type in {"revolute", "continuous"}:
        tf = tf @ transform(axis_angle_matrix(joint.axis, angle), np.zeros(3))
    return tf


def compute_link_transforms(
    children_by_parent: Dict[str, List[Joint]], joint_angles: Dict[str, float]
) -> Dict[str, np.ndarray]:
    link_transforms = {"base_link": np.eye(4)}
    stack = ["base_link"]

    while stack:
        parent = stack.pop()
        parent_tf = link_transforms[parent]
        for joint in children_by_parent.get(parent, []):
            angle = joint_angles.get(joint.name, 0.0)
            link_transforms[joint.child] = parent_tf @ joint_transform(joint, angle)
            stack.append(joint.child)

    return link_transforms


def point_from_transform(tf: np.ndarray, point: np.ndarray) -> np.ndarray:
    return (tf @ np.array([point[0], point[1], point[2], 1.0]))[:3]


def wheel_joint_position(
    link_transforms: Dict[str, np.ndarray], joints: Dict[str, Joint]
) -> np.ndarray:
    wheel_joint = joints[WHEEL_JOINT]
    return point_from_transform(link_transforms[wheel_joint.parent], wheel_joint.origin_xyz)


def rear_kinematics_xz(
    q1: float, q2: float, joints: Dict[str, Joint], children_by_parent: Dict[str, List[Joint]]
) -> np.ndarray:
    link_transforms = compute_link_transforms(children_by_parent, {REAR_JOINT_1: q1, REAR_JOINT_2: q2})
    return wheel_joint_position(link_transforms, joints)[[0, 2]]


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def solve_rear_ik(
    target_xz: np.ndarray,
    seed: np.ndarray,
    joints: Dict[str, Joint],
    children_by_parent: Dict[str, List[Joint]],
) -> np.ndarray:
    candidate_seeds = [
        seed,
        np.zeros(2),
        seed + np.array([0.4, -0.4]),
        seed + np.array([-0.4, 0.4]),
        seed + np.array([0.8, -0.8]),
        seed + np.array([-0.8, 0.8]),
        np.array([1.2, -1.2]),
        np.array([-1.2, 1.2]),
        np.array([2.4, -2.4]),
        np.array([-2.4, 2.4]),
    ]

    best: Tuple[float, float, np.ndarray] | None = None
    for candidate_seed in candidate_seeds:
        q = np.array([normalize_angle(candidate_seed[0]), normalize_angle(candidate_seed[1])])
        for _ in range(80):
            pos = rear_kinematics_xz(q[0], q[1], joints, children_by_parent)
            error = pos - target_xz
            if np.linalg.norm(error) < 1e-12:
                break

            jacobian = np.zeros((2, 2))
            step = 1e-6
            for axis_index in range(2):
                q_step = q.copy()
                q_step[axis_index] += step
                jacobian[:, axis_index] = (
                    rear_kinematics_xz(q_step[0], q_step[1], joints, children_by_parent) - pos
                ) / step

            damping = 1e-10
            delta = np.linalg.solve(
                jacobian.T @ jacobian + damping * np.eye(2),
                -jacobian.T @ error,
            )
            delta_norm = np.linalg.norm(delta)
            if delta_norm > 0.5:
                delta *= 0.5 / delta_norm
            q = np.array([normalize_angle(q[0] + delta[0]), normalize_angle(q[1] + delta[1])])

        final_error = np.linalg.norm(rear_kinematics_xz(q[0], q[1], joints, children_by_parent) - target_xz)
        score = final_error * 1000.0 + np.linalg.norm(q - seed) * 0.01 + np.linalg.norm(q) * 1e-6
        if best is None or score < best[0]:
            best = (score, final_error, q)

    if best is None or best[1] > 1e-8:
        raise RuntimeError(f"IK failed for target x-z position {target_xz.tolist()}")
    return best[2]


def equivalent_leg_properties(
    links: Dict[str, LinkInertial],
    link_transforms: Dict[str, np.ndarray],
    joints: Dict[str, Joint],
) -> Dict[str, float]:
    masses: List[float] = []
    centers: List[np.ndarray] = []
    inertia_tensors_base: List[np.ndarray] = []

    for link_name in LEG_LINKS:
        link = links[link_name]
        link_tf = link_transforms[link_name]
        inertial_rot = rpy_matrix(link.origin_rpy)
        base_to_inertial_rot = link_tf[:3, :3] @ inertial_rot

        masses.append(link.mass)
        centers.append(point_from_transform(link_tf, link.origin_xyz))
        inertia_tensors_base.append(base_to_inertial_rot @ link.inertia @ base_to_inertial_rot.T)

    total_mass = float(sum(masses))
    combined_com = sum(mass * center for mass, center in zip(masses, centers)) / total_mass

    inertia_axis_origin = joints[FRONT_JOINT].origin_xyz
    equivalent_iyy = 0.0
    for mass, center, inertia_base in zip(masses, centers, inertia_tensors_base):
        radius = center - inertia_axis_origin
        equivalent_iyy += inertia_base[1, 1] + mass * (radius[0] * radius[0] + radius[2] * radius[2])

    wheel_pos = wheel_joint_position(link_transforms, joints)
    hip_pos = joints[REAR_JOINT_1].origin_xyz

    return {
        "total_leg_mass_kg": total_mass,
        "com_x_m": combined_com[0],
        "com_y_m": combined_com[1],
        "com_z_m": combined_com[2],
        "inertia_axis_x_m": inertia_axis_origin[0],
        "inertia_axis_y_m": inertia_axis_origin[1],
        "inertia_axis_z_m": inertia_axis_origin[2],
        "wheel_joint_x_m": wheel_pos[0],
        "wheel_joint_y_m": wheel_pos[1],
        "wheel_joint_z_m": wheel_pos[2],
        "actual_joint_distance_xz_m": float(np.linalg.norm((wheel_pos - hip_pos)[[0, 2]])),
        "equivalent_com_distance_m": combined_com[2] - wheel_pos[2],
        "wheel_to_equivalent_com_z_m": combined_com[2] - wheel_pos[2],
        "equivalent_inertia_yy_kg_m2": equivalent_iyy,
    }


def ordered_indices_near_default(samples: List[float], default_distance: float) -> List[int]:
    return sorted(range(len(samples)), key=lambda index: abs(samples[index] - default_distance))


def format_float(value: float) -> str:
    return f"{value:.12g}"


def sample_distances() -> List[float]:
    return [0.10 + 0.05 * index for index in range(7)]


def calculate_rows(urdf_path: Path) -> List[Dict[str, float]]:
    links, joints, children_by_parent = parse_urdf(urdf_path)

    default_transforms = compute_link_transforms(children_by_parent, {})
    hip_pos = joints[REAR_JOINT_1].origin_xyz
    default_wheel_pos = wheel_joint_position(default_transforms, joints)
    default_delta_xz = (default_wheel_pos - hip_pos)[[0, 2]]
    default_distance = float(np.linalg.norm(default_delta_xz))
    if default_distance == 0.0:
        raise RuntimeError("Default rear wheel direction is zero length.")
    default_direction_xz = default_delta_xz / default_distance

    samples = sample_distances()
    solutions: Dict[int, np.ndarray] = {}
    for sample_index in ordered_indices_near_default(samples, default_distance):
        if solutions:
            nearest_index = min(solutions, key=lambda index: abs(samples[index] - samples[sample_index]))
            seed = solutions[nearest_index]
        else:
            seed = np.zeros(2)

        target_xz = hip_pos[[0, 2]] + default_direction_xz * samples[sample_index]
        solutions[sample_index] = solve_rear_ik(target_xz, seed, joints, children_by_parent)

    rows: List[Dict[str, float]] = []
    for sample_index, requested_distance in enumerate(samples):
        q1, q2 = solutions[sample_index]
        joint_angles = {REAR_JOINT_1: float(q1), REAR_JOINT_2: float(q2)}
        link_transforms = compute_link_transforms(children_by_parent, joint_angles)
        row = equivalent_leg_properties(links, link_transforms, joints)
        row.update(
            {
                "requested_joint_distance_xz_m": requested_distance,
                "base_link_left_rear1_link_joint_rad": float(q1),
                "left_rear1_link_left_rear2_link_joint_rad": float(q2),
                "target_direction_x": float(default_direction_xz[0]),
                "target_direction_z": float(default_direction_xz[1]),
            }
        )
        rows.append(row)

    return rows


def write_csv(rows: Iterable[Dict[str, float]], output_path: Path) -> None:
    rows = list(rows)
    if not rows:
        raise RuntimeError("No rows to write.")

    fieldnames = [
        "requested_joint_distance_xz_m",
        "actual_joint_distance_xz_m",
        "equivalent_com_distance_m",
        "wheel_to_equivalent_com_z_m",
        "equivalent_inertia_yy_kg_m2",
        "total_leg_mass_kg",
        "base_link_left_rear1_link_joint_rad",
        "left_rear1_link_left_rear2_link_joint_rad",
        "wheel_joint_x_m",
        "wheel_joint_y_m",
        "wheel_joint_z_m",
        "com_x_m",
        "com_y_m",
        "com_z_m",
        "inertia_axis_x_m",
        "inertia_axis_y_m",
        "inertia_axis_z_m",
        "target_direction_x",
        "target_direction_z",
    ]

    with output_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: format_float(float(row[key])) for key in fieldnames})


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=script_dir / "model" / "at_wheel_leg.urdf")
    parser.add_argument("--output", type=Path, default=script_dir / "left_leg_equivalent_params.csv")
    args = parser.parse_args()

    rows = calculate_rows(args.urdf)
    write_csv(rows, args.output)
    print(f"Wrote {len(rows)} rows to {args.output}")


if __name__ == "__main__":
    main()
