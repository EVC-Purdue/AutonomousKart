import numpy as np

from sim.validate import rollout_rmse


def test_rollout_rmse_zero_on_perfect_model():
    n = 1000
    bag = {
        "t": np.arange(n) / 60.0,
        "x": np.arange(n) * 0.1, "y": np.zeros(n), "yaw": np.zeros(n),
        "v": np.full(n, 6.0),
        "cmd_throttle": np.full(n, 60.0), "cmd_steer": np.zeros(n),
    }

    class PerfectSim:
        def reset(self, x, y, yaw):
            self.x, self.y, self.yaw, self.v = x, y, yaw, 6.0
        def step(self, target_mps, steer_deg, dt):
            self.x += self.v * dt
            return self.x, self.y, self.yaw, self.v

    sim = PerfectSim()
    rmse = rollout_rmse(sim, bag, anchor_indices=[0, 100, 500],
                        horizons_ticks=[6, 30, 60], dt=1 / 60)
    for metric, by_horizon in rmse.items():
        for h, v in by_horizon.items():
            assert v < 1e-6, f"{metric}@h{h} = {v}"
