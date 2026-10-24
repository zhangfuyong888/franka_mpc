"""窗口与视频共享的相机、目标和轨迹装饰；普通 headless 路径无需导入。"""
from collections import deque
import numpy as np


def overview_camera(model, config):
    """固定斜侧全景，围绕底座上方取景，给机械臂与工作空间预留边缘。"""
    import mujoco
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.lookat[:] = config['camera_lookat']
    camera.distance = config['camera_distance']
    camera.azimuth = config['camera_azimuth']
    camera.elevation = config['camera_elevation']
    return camera


class SceneOverlay:
    """按目标保存最近四条已结束轨迹，另保存当前段；展示不改变物理状态。"""
    def __init__(self):
        self.history = deque(maxlen=4)
        self.trail = deque(maxlen=350)
        self.goal_id = None
        self.last = -1.

    def observe(self, state, goal_id):
        if goal_id != self.goal_id:
            if self.trail:
                self.trail.append(state.p_ee.copy())
                self.history.append(self.trail)
            self.trail = deque([state.p_ee.copy()], maxlen=350)
            self.goal_id = goal_id
        if state.t-self.last >= 0.04-1e-10:
            self.last = state.t
            self.trail.append(state.p_ee.copy())

    def draw(self, scene, target, prediction):
        """在已有场景后添加装饰；Renderer 场景中的机械臂几何必须保留。"""
        import mujoco
        def sphere(p, radius, color):
            if scene.ngeom >= scene.maxgeom:
                return
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                              np.full(3, radius), np.asarray(p), np.eye(3).ravel(),
                              np.asarray(color, float))
            scene.ngeom += 1
        if target is not None:
            sphere(target, 0.018, [1, 0.2, 0.1, 0.8])
        for trail in self.history:
            for p in trail:
                sphere(p, 0.003, [0.2, 0.9, 0.2, 0.35])
        for p in self.trail:
            sphere(p, 0.003, [0.2, 0.9, 0.2, 0.6])
        for p in prediction:
            sphere(p, 0.006, [0.1, 0.4, 1., 0.7])
