"""Optional viewer, imported exclusively on the GUI path."""
from collections import deque
import time
import numpy as np


class Visualizer:
    def __init__(self, sim):
        import mujoco.viewer
        self.viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
        self.trail = deque(maxlen=350)
        self.last = -1.
        self.origin = time.perf_counter()

    def update(self, state, target, prediction):
        import mujoco
        if state.t-self.last < 0.04:
            return self.viewer.is_running()
        self.last = state.t
        self.trail.append(state.p_ee.copy())
        with self.viewer.lock():
            scene = self.viewer.user_scn
            scene.ngeom = 0
            def sphere(p, radius, color):
                if scene.ngeom >= scene.maxgeom:
                    return
                mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                    np.full(3, radius), np.asarray(p), np.eye(3).ravel(), np.asarray(color, float))
                scene.ngeom += 1
            if target is not None:
                sphere(target, 0.018, [1, 0.2, 0.1, 0.8])
            for p in self.trail:
                sphere(p, 0.003, [0.2, 0.9, 0.2, 0.6])
            for p in prediction:
                sphere(p, 0.006, [0.1, 0.4, 1., 0.7])
        self.viewer.sync()
        delay = state.t-(time.perf_counter()-self.origin)
        if delay > 0:
            time.sleep(delay)
        return self.viewer.is_running()

    def close(self):
        self.viewer.close()
