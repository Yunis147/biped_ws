"""Load precomputed, model-specific paths without running the offline IK solver."""
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import tempfile
import numpy as np
from . import quasi_static_walk as m

VERSION = 2
BUNDLED = Path(__file__).with_name('data')


def _json(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      default=lambda x: x.tolist() if hasattr(x, 'tolist') else str(x))


def fingerprint(settings, direction, initial):
    model = dict(legs=m.LEG, base=m.BASE, soles=m.SOLE)
    config = dict(profile=settings.profile(), landing_speed=settings.landing_speed,
                  segment_times=m.SEGMENT_TIME, time_scale=m.TIME_SCALE,
                  max_speed=m.MAX_JOINT_SPEED, max_acceleration=m.MAX_JOINT_ACCEL,
                  samples=m.TABLE_SAMPLES)
    data = dict(version=VERSION, model=model, config=config, direction=direction,
                initial=np.asarray(initial).tolist(), order=m.JOINT_ORDER,
                math=hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest())
    return hashlib.sha256(_json(data).encode()).hexdigest()


def cache_directory():
    return Path(os.environ.get('XDG_CACHE_HOME', str(Path.home()/'.cache')))/'biped_gait'


def _digest(arrays):
    h = hashlib.sha256()
    for name, array in sorted(arrays.items()):
        h.update(name.encode()); h.update(np.ascontiguousarray(array).tobytes())
    return h.hexdigest()


def save(plan, settings, direction, directory=None):
    directory = cache_directory() if directory is None else Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    key = fingerprint(settings, direction, plan.initial)
    frames = list(plan.keyframes)
    arrays = dict(initial=plan.initial, frames=np.array([[plan.keyframes[n][j] for j in m.JOINT_ORDER] for n in frames]),
                  tables=np.array([plan.tables[s] for s in m.SEGMENTS]),
                  durations=np.array([plan.durations[s] for s in m.SEGMENTS]))
    metadata = dict(key=key, frames=frames, metrics=plan.metrics, checksum=_digest(arrays))
    # An interrupted compile never replaces an existing usable trajectory.
    fd, temporary = tempfile.mkstemp(dir=directory, suffix='.npz')
    try:
        with os.fdopen(fd, 'wb') as out:
            np.savez_compressed(out, metadata=_json(metadata), **arrays)
        os.replace(temporary, directory/(key+'.npz'))
    finally:
        if os.path.exists(temporary): os.unlink(temporary)
    return directory/(key+'.npz')


def load(settings, direction, initial, directory=None):
    from .feedback_walk import Plan
    key = fingerprint(settings, direction, initial)
    directories = [Path(directory)] if directory is not None else [cache_directory(), BUNDLED]
    for folder in directories:
        path = folder/(key+'.npz')
        if not path.is_file(): continue
        try:
            with np.load(path, allow_pickle=False) as data:
                meta = json.loads(str(data['metadata']))
                arrays = {k: data[k] for k in ('initial', 'frames', 'tables', 'durations')}
            if meta['key'] != key or meta['checksum'] != _digest(arrays):
                raise ValueError('trajectory checksum mismatch')
            tables = arrays['tables']
            if (tables.shape != (len(m.SEGMENTS), m.TABLE_SAMPLES+1, 12)
                    or arrays['durations'].shape != (len(m.SEGMENTS),)
                    or arrays['frames'].shape != (len(meta['frames']), 12)
                    or not np.array_equal(arrays['initial'], initial)
                    or any(not np.all(np.isfinite(a)) for a in arrays.values())
                    or np.any(arrays['durations'] <= 0)
                    or np.any(tables < m.LO+m.LIMIT_MARGIN-1e-8)
                    or np.any(tables > m.HI-m.LIMIT_MARGIN+1e-8)):
                raise ValueError('invalid trajectory arrays')
            plan = Plan.__new__(Plan)
            plan.initial, plan.profile, plan.metrics = np.asarray(initial).copy(), settings.profile(), meta['metrics']
            plan.keyframes = {n: dict(zip(m.JOINT_ORDER, row)) for n,row in zip(meta['frames'], arrays['frames'])}
            plan.tables = dict(zip(m.SEGMENTS, tables))
            plan.durations = dict(zip(m.SEGMENTS, arrays['durations']))
            return plan
        except (ValueError, KeyError, OSError) as error:
            raise ValueError('invalid cached walking path '+str(path)+': '+str(error)) from error
    raise ValueError('No precomputed path matches the robot/model settings. Run: ros2 run biped_gait compile_walk')


def prepare(settings, direction, initial):
    from .feedback_walk import Plan
    try:
        return load(settings, direction, initial)
    except ValueError as error:
        if not str(error).startswith('No precomputed path'): raise
    plan = Plan(direction, initial, settings)
    save(plan, settings, direction)
    return plan


def main(args=None):
    import argparse
    from .feedback_walk import Settings, Plan
    parser = argparse.ArgumentParser(description='Compile and check walking paths offline; sends no robot commands')
    parser.add_argument('--config', help='walking ROS parameter YAML; omit for defaults')
    parser.add_argument('--output', help='output directory; default is the user cache')
    opts = parser.parse_args(args)
    values = {}
    if opts.config:
        import yaml
        raw = yaml.safe_load(Path(opts.config).read_text())['quasi_static_walker']['ros__parameters']
        values = {k:v for k,v in raw.items() if k in asdict(Settings())}
    settings = Settings(**values)
    for direction in ('forward', 'backward'):
        plan = Plan(direction, np.zeros(12), settings)
        print(direction, plan.metrics, flush=True)
        print('Saved', save(plan, settings, direction, opts.output), flush=True)


if __name__ == '__main__':
    main()
