"""URDF parser — stdlib-only, returns typed dataclasses."""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field


def _floats(s: str) -> list[float]:
    return [float(x) for x in s.strip().split()]


@dataclass
class Geometry:
    type: str  # box | cylinder | sphere | mesh
    size: list[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    radius: float = 0.1
    length: float = 0.1
    filename: str = ""
    scale: list[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])


@dataclass
class Pose:
    xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class VisualCollision:
    pose: Pose
    geometry: Geometry | None
    color: list[float] = field(default_factory=lambda: [0.58, 0.64, 0.74, 1.0])


@dataclass
class Link:
    name: str
    visuals: list[VisualCollision] = field(default_factory=list)
    collisions: list[VisualCollision] = field(default_factory=list)


@dataclass
class Joint:
    name: str
    type: str  # fixed | continuous | revolute | prismatic | floating | planar
    parent: str
    child: str
    pose: Pose = field(default_factory=Pose)
    axis: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0])
    limit_lower: float = 0.0
    limit_upper: float = 0.0


@dataclass
class Robot:
    name: str
    links: list[Link] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)

    def link_map(self) -> dict[str, Link]:
        return {lnk.name: lnk for lnk in self.links}

    def joint_map(self) -> dict[str, Joint]:
        return {j.name: j for j in self.joints}

    def parent_map(self) -> dict[str, str]:
        """child → parent link name."""
        return {j.child: j.parent for j in self.joints}

    def children_map(self) -> dict[str, list[str]]:
        kids: dict[str, list[str]] = {lnk.name: [] for lnk in self.links}
        for j in self.joints:
            kids[j.parent].append(j.child)
        return kids

    def root_link(self) -> str | None:
        children = {j.child for j in self.joints}
        for lnk in self.links:
            if lnk.name not in children:
                return lnk.name
        return None


def _parse_pose(elem) -> Pose:
    if elem is None:
        return Pose()
    return Pose(
        xyz=_floats(elem.get("xyz", "0 0 0")),
        rpy=_floats(elem.get("rpy", "0 0 0")),
    )


def _parse_geometry(elem) -> Geometry | None:
    if elem is None:
        return None
    child = next(
        (
            elem.find(t)
            for t in ("box", "cylinder", "sphere", "mesh")
            if elem.find(t) is not None
        ),
        None,
    )
    if child is None:
        return None
    tag = child.tag
    g = Geometry(type=tag)
    if tag == "box":
        g.size = _floats(child.get("size", "1 1 1"))
    elif tag == "cylinder":
        g.radius = float(child.get("radius", "0.1"))
        g.length = float(child.get("length", "0.1"))
    elif tag == "sphere":
        g.radius = float(child.get("radius", "0.1"))
    elif tag == "mesh":
        g.filename = child.get("filename", "")
        g.scale = _floats(child.get("scale", "1 1 1"))
    return g


def _parse_color(elem) -> list[float]:
    """Return [r, g, b, a] from <material><color rgba="..."/> or default."""
    mat = elem.find("material")
    if mat is not None:
        col = mat.find("color")
        if col is not None:
            return _floats(col.get("rgba", "0.58 0.64 0.74 1.0"))
    return [0.58, 0.64, 0.74, 1.0]


def _parse_vc(elem) -> VisualCollision:
    return VisualCollision(
        pose=_parse_pose(elem.find("origin")),
        geometry=_parse_geometry(elem.find("geometry")),
        color=_parse_color(elem),
    )


def parse_urdf(path: str) -> Robot:
    """Parse a URDF file and return a :class:`Robot`."""
    path = os.path.abspath(path)
    tree = ET.parse(path)
    root = tree.getroot()

    robot = Robot(name=root.get("name", ""))

    for link_el in root.findall("link"):
        lnk = Link(name=link_el.get("name", ""))
        for v in link_el.findall("visual"):
            lnk.visuals.append(_parse_vc(v))
        for c in link_el.findall("collision"):
            lnk.collisions.append(_parse_vc(c))
        robot.links.append(lnk)

    for joint_el in root.findall("joint"):
        p_el = joint_el.find("parent")
        c_el = joint_el.find("child")
        if p_el is None or c_el is None:
            continue
        j = Joint(
            name=joint_el.get("name", ""),
            type=joint_el.get("type", "fixed"),
            parent=p_el.get("link", ""),
            child=c_el.get("link", ""),
            pose=_parse_pose(joint_el.find("origin")),
        )
        axis_el = joint_el.find("axis")
        if axis_el is not None:
            j.axis = _floats(axis_el.get("xyz", "1 0 0"))
        limit_el = joint_el.find("limit")
        if limit_el is not None:
            j.limit_lower = float(limit_el.get("lower", "0"))
            j.limit_upper = float(limit_el.get("upper", "0"))
        robot.joints.append(j)

    return robot
