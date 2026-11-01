"""Optional offscreen preview of the actual MuJoCo controller, at 1x playback.

Run after sourcing ROS, from the workspace source folder, with a Python that has
MuJoCo and Pillow. Set MUJOCO_GL=egl (or osmesa) on a headless computer.
"""
from pathlib import Path
import math
import sys
import tempfile

import mujoco
import numpy as np
from PIL import Image, ImageDraw
import xacro

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/p) for p in ('biped_cpg', 'biped_gait', 'biped_mujoco')]
from biped_cpg.controller import Library, Walker
from biped_mujoco.sim_core import BipedSim


def main():
    package = ROOT/'robotv2_description'
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory)/'robot.xacro'
        source.write_text((package/'urdf/robot.urdf.xacro').read_text().replace(
            '$(find robotv2_description)', str(package)))
        urdf = xacro.process_file(str(source), mappings={'use_sim': 'false'}).toxml()
    sim = BipedSim(urdf, package, {'kp': 50., 'torque_fraction': .5})
    walker = Walker(Library())
    renderer = mujoco.Renderer(sim.model, 360, 480)
    camera = mujoco.MjvCamera()
    camera.distance, camera.azimuth, camera.elevation = .65, 125., -15.
    camera.lookat[:] = [0., 0., .14]
    # Preview lighting only; servo, contact and physical parameters are unchanged.
    sim.model.vis.headlight.ambient[:] = [.45, .45, .45]
    sim.model.vis.headlight.diffuse[:] = [.7, .7, .7]
    frames = []
    recording_at = None
    next_frame = 0.
    try:
        for tick in range(int(100/sim.model.opt.timestep)):
            if tick % 10 == 0:
                if sim.time >= 2 and not walker.active:
                    walker.command('forward', sim.time)
                target = walker.tick(sim.time)
                if target is not None:
                    sim.set_targets(target)
                if walker.state == 'walking' and walker.cpg.right >= math.pi and recording_at is None:
                    recording_at, next_frame = sim.time, sim.time
            sim.step()
            if sim.fallen() or walker.state == 'fault':
                raise RuntimeError('preview run failed')
            if recording_at is not None and sim.time >= next_frame:
                camera.lookat[0] = sim.base_pose()[0][0]
                renderer.update_scene(sim.data, camera=camera)
                picture = Image.fromarray(renderer.render())
                draw = ImageDraw.Draw(picture)
                roll = math.degrees(math.atan2(sim.data.xmat[sim.base_id].reshape(3, 3)[2, 1],
                                               sim.data.xmat[sim.base_id].reshape(3, 3)[2, 2]))
                draw.rectangle((0, 0, 480, 36), fill=(20, 29, 42))
                draw.text((10, 5), 'biped_cpg | MuJoCo | 1x playback | no sensor feedback', fill='white')
                draw.text((10, 20), f'Actual body roll {roll:+.1f} deg | simulated servo kp=50, torque=50%', fill='white')
                frames.append(picture)
                next_frame += .2
            if recording_at is not None and sim.time >= recording_at+walker.plan.period:
                break
        output = ROOT/'biped_cpg/assets/cpg_walk.gif'
        output.parent.mkdir(exist_ok=True)
        frames[0].save(output, save_all=True, append_images=frames[1:], duration=200, loop=0, optimize=True)
        print(f'{output}: {len(frames)} frames at 5 fps')
    finally:
        renderer.close()
        sim._tmp.cleanup()


if __name__ == '__main__':
    main()
