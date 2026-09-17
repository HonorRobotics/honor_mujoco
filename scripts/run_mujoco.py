from __future__ import annotations
import threading
import time
from typing import Callable, Optional
import mujoco
import mujoco.viewer
import config
from ros2_bridge import ElasticBand, Ros2Bridge

VIEWER_WARMUP_S = 0.2
BAND_ANCHOR_BODY = "torso_link"


class RateLimiter:
    def __init__(self, period: float) -> None:
        self._period = period
        self._started = time.perf_counter()

    def tick(self) -> None:
        remaining = self._period - (time.perf_counter() - self._started)
        if remaining > 0:
            time.sleep(remaining)
        self._started = time.perf_counter()


class HumanoidSimulation:
    def __init__(self, scene_path: str, timestep: float) -> None:
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.model.opt.timestep = timestep
        self.data = mujoco.MjData(self.model)

        self._lock = threading.Lock()
        self._band: Optional[ElasticBand] = (
            ElasticBand() if config.ENABLE_ELASTIC_BAND else None
        )
        self._band_body_id = (
            self.model.body(BAND_ANCHOR_BODY).id if self._band is not None else -1
        )

    @property
    def key_callback(self) -> Optional[Callable[[int], None]]:
        return None if self._band is None else self._band.key_callback

    def _pull_elastic_band(self) -> None:
        if self._band is None or not self._band.enable:
            return
        self.data.xfrc_applied[self._band_body_id, :3] = self._band.advance(
            self.data.qpos[:3], self.data.qvel[:3]
        )

    def physics_loop(self, viewer, bridge: Ros2Bridge) -> None:
        rate = RateLimiter(self.model.opt.timestep)

        while viewer.is_running():
            with self._lock:
                bridge.update_torque()
                self._pull_elastic_band()
                mujoco.mj_step(self.model, self.data)
                mujoco.mj_rnePostConstraint(self.model, self.data)

            rate.tick()

    def render_loop(self, viewer) -> None:
        while viewer.is_running():
            with self._lock:
                viewer.cam.lookat[:] = self.data.qpos[0:3]
                viewer.sync()
            time.sleep(config.VIEWER_DT)


def main() -> None:
    sim = HumanoidSimulation(config.ROBOT_SCENE, config.SIMULATE_DT)

    with mujoco.viewer.launch_passive(
        sim.model, sim.data, key_callback=sim.key_callback
    ) as viewer:
        time.sleep(VIEWER_WARMUP_S)

        bridge = Ros2Bridge(sim.model, sim.data)
        bridge.print_scene_information()

        workers = [
            threading.Thread(target=sim.render_loop, args=(viewer,), name="render"),
            threading.Thread(target=sim.physics_loop, args=(viewer, bridge), name="physics"),
        ]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join()


if __name__ == "__main__":
    main()
