"""Offline regression checks; no ROS, simulator or robot connection required.

Run: python3 -B -m unittest discover -s biped_gait/test -v
"""
import importlib.util
from pathlib import Path
import struct
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'biped_gait'))
from biped_gait.robot_model import load_model


class FakeNode:
    def __init__(self, name='test'):
        self.seconds = 0.0
        self.messages = []

    def create_publisher(self, *args):
        return SimpleNamespace(publish=lambda msg: self.messages.append(np.array(msg.data)))

    def create_subscription(self, *args):
        pass

    def create_timer(self, *args):
        pass

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(self.seconds * 1e9)))

    def get_logger(self):
        return SimpleNamespace(**{name: lambda msg: None for name in ('info', 'warn', 'error')})


ros = {name: ModuleType(name) for name in ('rclpy', 'rclpy.node', 'std_msgs', 'std_msgs.msg')}
ros['rclpy.node'].Node = FakeNode
ros['std_msgs.msg'].Float64MultiArray = SimpleNamespace
ros['std_msgs.msg'].String = SimpleNamespace
spec = importlib.util.spec_from_file_location('walker_under_test', ROOT / 'biped_gait/biped_gait/quasi_static_walk.py')
gait = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, ros):
    spec.loader.exec_module(gait)


def urdf_rotation(axis, angle):
    """Independent Rodrigues construction for the URDF model."""
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


class WalkingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = ET.parse(ROOT / 'robotv2_description/urdf/robot.urdf.xacro').getroot()
        cls.gaits = {direction: gait.build_tables(direction) for direction in ('forward', 'backward')}

    def independent_fk(self, row):
        q = dict(zip(gait.JOINT_ORDER, row))
        transforms = {'base_link': np.eye(4)}
        pending = list(self.robot.findall('joint'))
        while pending:
            for joint in list(pending):
                parent = joint.find('parent').get('link')
                if parent not in transforms:
                    continue
                origin = joint.find('origin')
                np.testing.assert_allclose(np.fromstring(origin.get('rpy'), sep=' '), 0)
                T = np.eye(4)
                T[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
                if joint.get('type') == 'revolute':
                    T[:3, :3] = urdf_rotation(np.fromstring(joint.find('axis').get('xyz'), sep=' '), q[joint.get('name')])
                transforms[joint.find('child').get('link')] = transforms[parent] @ T
                pending.remove(joint)
        com = np.zeros(3)
        total = 0.0
        for link in self.robot.findall('link'):
            inertial = link.find('inertial')
            mass = float(inertial.find('mass').get('value'))
            local = np.append(np.fromstring(inertial.find('origin').get('xyz'), sep=' '), 1)
            com += mass * (transforms[link.get('name')] @ local)[:3]
            total += mass
        feet = {}
        for side in 'rl':
            T = transforms[f'{side}_foot_link_1']
            local = np.array([[x, y, gait.SOLE[side]['z'], 1] for x, y in gait.SOLE[side]['poly']])
            feet[side] = (T @ local.T).T[:, :3]
        return com / total, feet, {s: transforms[f'{s}_foot_link_1'][:3, :3] for s in 'rl'}

    def test_loaded_model_matches_current_urdf(self):
        total = gait.BASE['mass']
        self.assertEqual(total, float(self.robot.find("link[@name='base_link']/inertial/mass").get('value')))
        for side in 'rl':
            for data in gait.LEG[side]:
                joint = self.robot.find(f"joint[@name='{data['name']}']")
                child = joint.find('child').get('link')
                inertial = self.robot.find(f"link[@name='{child}']/inertial")
                self.assertEqual(data['mass'], float(inertial.find('mass').get('value')))
                np.testing.assert_allclose(data['com'], np.fromstring(inertial.find('origin').get('xyz'), sep=' '))
                np.testing.assert_allclose(data['xyz'], np.fromstring(joint.find('origin').get('xyz'), sep=' '))
                np.testing.assert_allclose(data['axis'], np.fromstring(joint.find('axis').get('xyz'), sep=' '))
                self.assertEqual((data['lo'], data['hi']), tuple(float(joint.find('limit').get(k)) for k in ('lower', 'upper')))
                total += data['mass']
        self.assertAlmostEqual(total, 1.291500, places=6)

    def test_contact_area_comes_from_scaled_bottom_vertices(self):
        # Parse STL triangles independently, including mesh scale AND offset.
        for side in 'rl':
            collision = self.robot.find(f"link[@name='{side}_foot_link_1']/collision")
            mesh = collision.find('geometry/mesh')
            path = ROOT / mesh.get('filename').removeprefix('package://')
            data = path.read_bytes()
            count = struct.unpack_from('<I', data, 80)[0]
            vertices = np.array([struct.unpack_from('<9f', data, 84 + i*50 + 12)
                                 for i in range(count)]).reshape(-1, 3)
            vertices *= np.fromstring(mesh.get('scale'), sep=' ')
            vertices += np.fromstring(collision.find('origin').get('xyz'), sep=' ')
            z = vertices[:, 2].min()
            bottom = vertices[np.abs(vertices[:, 2] - z) < 1e-7]
            expected = sorted(map(tuple, np.unique(bottom[:, :2], axis=0)))
            self.assertEqual(sorted(map(tuple, gait.SOLE[side]['poly'])), expected)
            self.assertAlmostEqual(gait.SOLE[side]['z'], z, places=12)
        self.assertAlmostEqual(gait.ZERO_HEIGHT, .2738826747, places=9)
        self.assertAlmostEqual(gait.BODY_HEIGHT, gait.ZERO_HEIGHT - .040)

    def test_future_mass_and_mesh_edits_are_loaded_without_copied_constants(self):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory)
            (package / 'urdf').mkdir()
            (package / 'meshes').symlink_to(ROOT / 'robotv2_description/meshes', target_is_directory=True)
            robot = ET.fromstring(ET.tostring(self.robot))
            robot.find("link[@name='base_link']/inertial/mass").set('value', '0.425100')
            mesh = robot.find("link[@name='r_foot_link_1']/collision/geometry/mesh")
            scale = np.fromstring(mesh.get('scale'), sep=' ')
            scale[2] *= 1.01
            mesh.set('scale', ' '.join(map(str, scale)))
            path = package / 'urdf/robot.urdf.xacro'
            ET.ElementTree(robot).write(path)
            legs, base, soles, _ = load_model(package)
            self.assertAlmostEqual(base['mass'] + sum(j['mass'] for s in 'rl' for j in legs[s]), 1.3915)
            self.assertNotAlmostEqual(soles['r']['z'], gait.SOLE['r']['z'], places=5)
            # A changed mechanism must fail clearly, rather than use old IK assumptions.
            robot.find("joint[@name='l_knee_joint']/axis").set('xyz', '0 1 0')
            ET.ElementTree(robot).write(path)
            with self.assertRaisesRegex(ValueError, 'joint axis changed'):
                load_model(package)

    def test_floor_and_balance_with_independent_urdf_model(self):
        for direction, (kf, tables) in self.gaits.items():
            for seg, table in tables.items():
                loaded = gait.STANCE.get(seg[1])
                spacing = None
                for index in range(len(table) - 1):
                    # Include the values actually interpolated by the publisher.
                    for fraction in (0.0, 0.5):
                        pose = gait.table_pose(table, (index + fraction) / gait.TABLE_SAMPLES)
                        com, feet, rotations = self.independent_fk(pose)
                        model_com, model_feet = gait._com_and_feet(dict(zip(gait.JOINT_ORDER, pose)))
                        np.testing.assert_allclose(com, model_com, atol=1e-12)
                        for side in 'rl':
                            np.testing.assert_allclose(feet[side], model_feet[side], atol=1e-12)
                        rotation = rotations[loaded or 'l'].T
                        com = rotation @ com
                        feet = {s: f @ rotation.T for s, f in feet.items()}
                        for R in rotations.values():
                            np.testing.assert_allclose(rotation @ R, np.eye(3), atol=2e-6)
                        if loaded:
                            np.testing.assert_allclose(com[:2], feet[loaded].mean(0)[:2], atol=2e-6)
                        points = np.concatenate([feet[s] for s in ([loaded] if loaded else 'rl')])
                        # Independently test every support-polygon edge.
                        poly = gait._hull(points[:, :2])
                        for i, a in enumerate(poly):
                            edge = np.array(poly[(i + 1) % len(poly)]) - a
                            distance = np.dot(com[:2] - a, [-edge[1], edge[0]]) / np.linalg.norm(edge)
                            self.assertGreater(distance, .010, (direction, seg))
                        ground = points[:, 2].mean()
                        self.assertLess(np.ptp(points[:, 2]), .000002)
                        self.assertGreaterEqual(min(f[:, 2].min() for f in feet.values()), ground - .000002)
                        if not loaded:
                            current = feet['r'].mean(0) - feet['l'].mean(0)
                            if spacing is None:
                                spacing = current
                            np.testing.assert_allclose(current, spacing, atol=.00002)
            self.assertTrue(gait.report(direction, kf, tables, lambda msg: None))

    def test_landing_is_vertical_and_keeps_both_knees_bent(self):
        for kf, tables in self.gaits.values():
            for swing in ('right', 'left'):
                side = swing[0]
                table = tables[(f'swing_{swing}', f'place_{swing}')]
                stance = 'l' if side == 'r' else 'r'
                positions = []
                for row in table:
                    _, feet, rotations = self.independent_fk(row)
                    relative = feet[side].mean(0) - feet[stance].mean(0)
                    positions.append(rotations[stance].T @ relative)
                positions = np.array(positions)
                np.testing.assert_allclose(positions[:, :2], np.broadcast_to(positions[0, :2], positions[:, :2].shape), atol=1e-7)
                self.assertTrue(np.all(np.diff(positions[:, 2]) <= 1e-7))
                self.assertAlmostEqual(positions[-1, 2], 0., places=6)
                self.assertAlmostEqual(positions[0, 2] - positions[-1, 2], .008, places=6)
                for leg in 'rl':
                    self.assertLess(kf[f'place_{swing}'][f'{leg}_knee_joint'], -.12)

    def test_limits_direction_and_continuity_at_every_phase(self):
        for direction, (kf, tables) in self.gaits.items():
            for (a, b), table in tables.items():
                self.assertTrue(np.all(np.isfinite(table)))
                self.assertTrue(np.all(table >= gait.LO + gait.LIMIT_MARGIN - 1e-8))
                self.assertTrue(np.all(table <= gait.HI - gait.LIMIT_MARGIN + 1e-8))
                np.testing.assert_allclose(table[0], [kf[a][n] for n in gait.JOINT_ORDER], atol=1e-6)
                if b != 'neutral':
                    np.testing.assert_allclose(table[-1], [kf[b][n] for n in gait.JOINT_ORDER], atol=1e-6)
                # Regression: old placement forced a roughly 5-degree last-row jump.
                self.assertLess(np.max(np.abs(table[-1] - table[-2])), np.deg2rad(.25), (a, b))
            sign = 1 if direction == 'forward' else -1
            for phase, leading in [('place_right', 'r'), ('place_left', 'l')]:
                trailing = 'l' if leading == 'r' else 'r'
                _, feet, rotations = self.independent_fk(np.array([kf[phase][n] for n in gait.JOINT_ORDER]))
                delta = rotations[leading].T @ (feet[leading].mean(0) - feet[trailing].mean(0))
                self.assertAlmostEqual(delta[0] * sign, gait.STEP_LENGTH[direction], places=6)

    def test_invalid_targets_and_double_support_errors_are_rejected(self):
        seed = self.gaits['forward'][0]['ready']
        with self.assertRaises(ValueError):
            gait._leg_ik(seed, 'r', np.array([0, -.05, -.4]))
        kf, original = self.gaits['forward']
        tables = {key: value.copy() for key, value in original.items()}
        tables[('place_right', 'lean_right')][40, gait.JOINT_ORDER.index('r_knee_joint')] -= .1
        self.assertFalse(gait.report('forward', kf, tables, lambda msg: None))

    def test_planned_body_lean_and_motion_limits(self):
        for direction, (kf, tables) in self.gaits.items():
            for name, sign in (('lean_left', -1), ('lean_right', 1)):
                _, _, rotations = self.independent_fk([kf[name][n] for n in gait.JOINT_ORDER])
                body = rotations['l'].T
                roll = np.arctan2(body[2, 1], body[2, 2])
                self.assertAlmostEqual(roll, sign * gait.BODY_ROLL, places=6)
            for (a, b), table in tables.items():
                duration = gait.segment_duration(b, table)
                values = np.array([gait.table_pose(table, u) for u in np.linspace(0, 1, 2001)])
                dt = duration / 2000
                self.assertLessEqual(np.max(np.abs(np.diff(values, axis=0))) / dt,
                                     gait.MAX_JOINT_SPEED + 1e-5)
                self.assertLessEqual(np.max(np.abs(np.diff(values, n=2, axis=0))) / dt**2,
                                     gait.MAX_JOINT_ACCEL + 1e-5)
                # Each phase really starts/stops; there is no jump in velocity at a dwell.
                for edge in (0., 1.):
                    u = edge + (1e-6 if edge == 0 else -1e-6)
                    speed = np.max(np.abs(gait.table_pose(table, u) - gait.table_pose(table, edge))) / (duration * 1e-6)
                    self.assertLess(speed, .0001)

    def test_unreachable_settings_fail_before_command_generation(self):
        with patch.object(gait, 'BODY_ROLL', 0.):
            with self.assertRaises(ValueError):
                gait.build_tables('forward')
        with self.assertRaises(ValueError):
            gait.build_tables('sideways')

    def make_node(self):
        node = gait.QuasiStaticWalker.__new__(gait.QuasiStaticWalker)
        FakeNode.__init__(node)
        node.pub = node.create_publisher()
        node.keyframes = {d: kf for d, (kf, tables) in self.gaits.items()}
        node.tables = {d: tables for d, (kf, tables) in self.gaits.items()}
        node.neutral = np.zeros(12)
        node.state = 'idle'
        node.direction = 'forward'
        node.stop_req = False
        node._dwell_until = None
        node.cur = node.nxt = node.table = None
        node.T = 1.0
        node.t0 = 0.0
        return node

    def advance(self, node):
        node.seconds = (node._dwell_until if node._dwell_until is not None else node.t0 + node.T) + .000001
        node._tick()

    def at_landing(self, node):
        node.walk_cmd_cb(SimpleNamespace(data='start'))
        for _ in range(20):
            self.advance(node)
            if node.nxt == 'place_right' and node._dwell_until is not None:
                return
        self.fail('did not reach the landing dwell')

    def test_stop_during_landing_dwell_holds_and_restart_preserves_stride(self):
        node = self.make_node()
        self.at_landing(node)
        landed = node.messages[-1].copy()
        node.seconds += .5
        node._tick()
        np.testing.assert_array_equal(node.messages[-1], landed)
        node.walk_cmd_cb(SimpleNamespace(data='stop'))
        self.advance(node)
        self.assertEqual((node.cur, node.nxt), ('place_right', 'neutral'))
        self.advance(node)
        self.assertEqual(node.state, 'idle')
        stopped = node.messages[-1].copy()
        node.seconds += 1
        node._tick()
        np.testing.assert_array_equal(node.messages[-1], stopped)
        for direction in ('forward', 'backward'):
            kf, tables = gait.build_tables(direction, stopped)
            self.assertTrue(gait.report(direction, kf, tables, lambda msg: None))
            np.testing.assert_allclose(tables[('neutral', 'ready')][0], stopped, atol=1e-6)
        node.walk_cmd_cb(SimpleNamespace(data='backward'))
        self.assertEqual(node.state, 'walking')
        np.testing.assert_allclose(node.table[0], stopped, atol=1e-6)

    def test_stop_mid_lift_finishes_landing_and_timer_delay_does_not_skip(self):
        node = self.make_node()
        node.walk_cmd_cb(SimpleNamespace(data='start'))
        while node.nxt != 'lift_right_1st':
            self.advance(node)
        node.walk_cmd_cb(SimpleNamespace(data='stop'))
        node.seconds += 100  # a delayed callback must still send this phase's endpoint
        endpoint = node.table[-1].copy()
        node._tick()
        np.testing.assert_array_equal(node.messages[-1], endpoint)
        self.assertEqual(node.nxt, 'swing_right')
        for _ in range(8):
            self.advance(node)
            if node.state == 'idle':
                break
        self.assertEqual(node.state, 'idle')
        self.assertEqual(node.cur, 'place_right')

    def test_stop_can_be_cancelled_during_settle(self):
        node = self.make_node()
        self.at_landing(node)
        node.walk_cmd_cb(SimpleNamespace(data='stop'))
        node.walk_cmd_cb(SimpleNamespace(data='start'))
        self.advance(node)
        self.assertEqual((node.cur, node.nxt), ('place_right', 'lean_right'))

    def test_failed_startup_check_does_not_start_publishing(self):
        with patch.object(gait, 'build_tables', return_value=self.gaits['forward']), \
                patch.object(gait, 'report', return_value=False), \
                patch.object(FakeNode, 'create_timer') as timer:
            with self.assertRaises(ValueError):
                gait.QuasiStaticWalker()
            timer.assert_not_called()


if __name__ == '__main__':
    unittest.main()
