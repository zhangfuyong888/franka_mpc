"""Optional viewer, imported exclusively on the GUI path."""
import time
from .rendering import overview_camera, SceneOverlay


class Visualizer:
    def __init__(self, sim, config, overlay=None):
        import mujoco.viewer
        self.viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
        self.overlay = overlay if overlay is not None else SceneOverlay()
        with self.viewer.lock():
            camera = overview_camera(sim.model, config)
            self.viewer.cam.lookat[:] = camera.lookat
            self.viewer.cam.distance = camera.distance
            self.viewer.cam.azimuth = camera.azimuth
            self.viewer.cam.elevation = camera.elevation
        self.last = -1.
        self.origin = time.perf_counter()

    def update(self, state, target, prediction, goal_id=None):
        self.overlay.observe(state, goal_id)
        if state.t-self.last < 0.04:
            return self.viewer.is_running()
        self.last = state.t
        with self.viewer.lock():
            scene = self.viewer.user_scn
            scene.ngeom = 0
            self.overlay.draw(scene, target, prediction)
        self.viewer.sync()
        delay = state.t-(time.perf_counter()-self.origin)
        if delay > 0:
            time.sleep(delay)
        return self.viewer.is_running()

    def close(self):
        self.viewer.close()
