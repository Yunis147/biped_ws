"""Read the walking model from the description package, including scaled soles.

The present xacro contains concrete link/joint elements. Reject unsupported joint
layouts rather than silently applying the flat-foot solver to a different robot.
No ROS node, controller, or sensor is started by this module.
"""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


def description_path():
    # A sourced installation is authoritative. Direct source execution also works.
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
        try:
            return Path(get_package_share_directory('robotv2_description'))
        except PackageNotFoundError:
            pass
    except ImportError:
        pass
    source = Path(__file__).resolve().parents[2] / 'robotv2_description'
    if source.is_dir():
        return source
    raise ValueError('robotv2_description is missing; build and source the workspace first')


def hull(points):
    points = sorted(set(map(tuple, points)))
    if len(points) < 3:
        return points

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _vector(element, key, default='0 0 0'):
    value = np.fromstring(element.get(key, default), sep=' ')
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise ValueError(f'invalid {key} in robot description')
    return value


def _inertial(link):
    inertial = link.find('inertial')
    if inertial is None:
        raise ValueError(f"missing inertial for {link.get('name')}")
    mass = float(inertial.find('mass').get('value'))
    origin = inertial.find('origin')
    if not np.allclose(_vector(origin, 'rpy'), 0, atol=1e-12):
        raise ValueError('rotated inertia frames are not supported by this walker')
    if not np.isfinite(mass) or mass < 0:
        raise ValueError('invalid link mass')
    i = inertial.find('inertia')
    inertia = np.array([
        [float(i.get('ixx')), float(i.get('ixy')), float(i.get('ixz'))],
        [float(i.get('ixy')), float(i.get('iyy')), float(i.get('iyz'))],
        [float(i.get('ixz')), float(i.get('iyz')), float(i.get('izz'))]])
    if not np.all(np.isfinite(inertia)):
        raise ValueError('invalid inertia tensor')
    return dict(mass=mass, com=_vector(origin, 'xyz'), inertia=inertia)


def sole_from_collision(link, package):
    """Convex contact outline at the lowest plane of this link's binary STL."""
    collisions = link.findall('collision')
    if len(collisions) != 1:
        raise ValueError('expected one collision mesh per foot')
    collision = collisions[0]
    origin = collision.find('origin')
    if not np.allclose(_vector(origin, 'rpy'), 0, atol=1e-12):
        raise ValueError('rotated foot collision mesh is not supported')
    mesh = collision.find('geometry/mesh')
    prefix = 'package://robotv2_description/'
    if mesh is None or not mesh.get('filename', '').startswith(prefix):
        raise ValueError('expected a robotv2_description foot STL')
    path = package / mesh.get('filename')[len(prefix):]
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f'invalid binary STL: {path}')
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * count or not count:
        raise ValueError(f'expected a binary triangle STL: {path}')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    vertices = np.frombuffer(data, dtype=dtype, offset=84)['vertices'].reshape(-1, 3).astype(float)
    scale = _vector(mesh, 'scale', '1 1 1')
    if np.any(scale <= 0):
        raise ValueError('foot mesh scale must be positive')
    vertices = vertices * scale + _vector(origin, 'xyz')
    z = float(vertices[:, 2].min())
    bottom = vertices[np.abs(vertices[:, 2] - z) < 1e-7]
    poly = hull(bottom[:, :2])
    if len(poly) < 3:
        raise ValueError('foot mesh has no flat contact area at its lowest plane')
    # The current sole is a rectangle. Its centre and inset distances have a
    # simple unambiguous meaning; reject a changed sole shape for review.
    p = np.array(poly)
    if len(poly) != 4 or not all(np.count_nonzero(np.abs(e) > 1e-8) == 1
                                 for e in np.roll(p, -1, axis=0) - p):
        raise ValueError('this walker requires rectangular, axis-aligned soles')
    return dict(z=z, poly=poly)


def load_model(package=None):
    package = description_path() if package is None else Path(package)
    path = package / 'urdf/robot.urdf.xacro'
    robot = ET.parse(path).getroot()
    links = {link.get('name'): link for link in robot.findall('link')}
    joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    legs, soles = {}, {}
    used = {'base_link'}
    for side, sign in (('r', -1), ('l', 1)):
        names = [f'{side}_hip_yaw'] + [f'{side}_{name}_joint' for name in
                 ('hip_roll', 'hip_pitch', 'knee', 'ankle_roll', 'ankle_pitch')]
        axes = [(0, 0, sign), (sign, 0, 0), (0, 1, 0), (0, -1, 0), (sign, 0, 0), (0, 1, 0)]
        parent = 'base_link'
        legs[side] = []
        for name, axis in zip(names, axes):
            j = joints[name]
            if j.get('type') != 'revolute' or j.find('parent').get('link') != parent:
                raise ValueError(f'unsupported kinematic chain at {name}')
            origin = j.find('origin')
            if not np.allclose(_vector(origin, 'rpy'), 0, atol=1e-12):
                raise ValueError(f'rotated joint origin at {name} is not supported')
            if not np.allclose(_vector(j.find('axis'), 'xyz'), axis, atol=1e-12):
                raise ValueError(f'joint axis changed at {name}; review the flat-foot solver')
            parent = j.find('child').get('link')
            used.add(parent)
            limit = j.find('limit')
            legs[side].append(dict(name=name, xyz=_vector(origin, 'xyz'), axis=axis,
                                   lo=float(limit.get('lower')), hi=float(limit.get('upper')),
                                   **_inertial(links[parent])))
        soles[side] = sole_from_collision(links[parent], package)
    # A new massive link must never be silently omitted from the CoM sum.
    for name, link in links.items():
        if name not in used and _inertial(link)['mass'] != 0:
            raise ValueError(f'unmodelled massive link: {name}')
    return legs, _inertial(links['base_link']), soles, path
