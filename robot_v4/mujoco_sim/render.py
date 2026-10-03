"""Render a few representative clips to MP4 (small files: 640x400, ~30 fps).

Run: ../../.venv/bin/python render.py   (writes clips/*.mp4)
"""
import math
import os
import sys

import imageio.v2 as imageio
import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'clips')
COLORS = {'V4': (0.15, 0.35, 0.8, 1), 'V2': (0.95, 0.55, 0.1, 1), 'R3': (0.2, 0.65, 0.3, 1)}


def paint(m, kinds):
    for g in range(m.ngeom):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
        if name.startswith(('A_', 'B_')):
            kind = kinds[name[0]]
            rgba = COLORS[kind]
            if '_w' in name and name.endswith('_g') and name[2] == 'w':
                rgba = (0.1, 0.1, 0.1, 1)
            if any(k in name for k in ('tooth', 'disc', 'hub')):
                rgba = (0.85, 0.1, 0.1, 1)
            m.geom_rgba[g] = rgba
        elif name.startswith('wall'):
            m.geom_rgba[g] = (0.6, 0.6, 0.65, 0.35)


def render(rec, kinds, path, fps=30, slow=1.0, size=(640, 400), dist=0.9, elev=-28, azim=135):
    m = mujoco.MjModel.from_xml_string(rec['xml'])
    paint(m, kinds)
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, size[1], size[0])
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.elevation, cam.azimuth = dist, elev, azim
    a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'A')
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'B')
    frames = []
    for t, q in rec['frames']:
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        cam.lookat[:] = (d.xpos[a] + d.xpos[b]) / 2 + np.array([0, 0, 0.03])
        r.update_scene(d, cam)
        frames.append(r.render())
    os.makedirs(OUT, exist_ok=True)
    imageio.mimwrite(path, frames, fps=fps, codec='libx264', quality=6, macro_block_size=8)
    return os.path.getsize(path)


CLIPS = [
    # name, A, B, seed, kwargs, frame_dt, slow-motion note
    ('v4_vs_v2_headon_slowmo', 'V4', 'V2', 3, dict(t_end=1.2, prespin=True, scripted=True,
                                                   start=[((0.0, -0.25), 0.0), ((0.0, 0.25), math.pi)]), 1 / 240),
    ('v4_vs_r3_headon_slowmo', 'V4', 'R3', 3, dict(t_end=1.2, prespin=True, scripted=True,
                                                   start=[((0.0, -0.25), 0.0), ((0.0, 0.25), math.pi)]), 1 / 240),
    ('v4_vs_v2_engagement', 'V4', 'V2', 2, dict(t_end=6.0), 1 / 30),
    ('v2_vs_r3_engagement', 'V2', 'R3', 2, dict(t_end=6.0, dt=1e-4), 1 / 30),
]

if __name__ == '__main__':
    for name, A, B, seed, kw, fdt in CLIPS:
        rec = {'frame_dt': fdt}
        s = sim.engagement(A, B, seed, record=rec, **kw)
        size = render(rec, {'A': A, 'B': B}, os.path.join(OUT, name + '.mp4'))
        print(name, sim.classify(s) if 'error' not in s else s, '%.1f MB' % (size / 1e6), len(rec['frames']), 'frames')
