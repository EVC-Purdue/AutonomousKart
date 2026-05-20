"""Physical kart parameters used by the simulator.

This is the SOURCE OF TRUTH for the kart's physics: it mirrors values from
``src/autonomous_kart/autonomous_kart/params/pathfinder.yaml``.  The sim uses
these to model the kart; bag data is treated as ground truth to compare
against (NOT as input to fit the bicycle).

If you change the kart hardware (wheelbase, max steer, etc.) update this file
and the yaml together — they MUST agree.

A few values aren't documented in the yaml because they're effective model
parameters (motor lag, mechanical slop) rather than direct mechanical limits.
Those defaults are engineering estimates; document any change you make.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass
class KartPhysics:
    # --- direct from pathfinder.yaml ----------------------------------------
    v_max_mps: float = 12.0                # max kart speed
    wheelbase_m: float = 1.05              # rear-axle to front-axle
    steer_max_deg: float = 60.0            # physical front-wheel limit
    steer_rate_max_degps: float = 180.0    # max steering slew rate
    a_max_mps2: float = 2.0                # max forward accel (MPC planning bound)
    a_min_mps2: float = -3.0               # max braking decel (MPC planning bound)
    a_lat_max_mps2: float = 4.0            # tire-grip lateral cap (MPC planning bound)
    # --- engineering estimates (NOT in yaml) --------------------------------
    accel_tau: float = 0.6                 # first-order throttle lag (s)
    brake_tau: float = 0.4                 # first-order brake lag (s)
    steer_slop_deg: float = 0.0            # mechanical play in steering linkage
    # --- conversion factors -------------------------------------------------
    # steer_gain: ratio of "actual wheel angle" / "planner's commanded angle".
    # 1.0 means cmd in degrees == wheel angle in degrees.  Set <1 if your
    # e_comms layer scales the command (e.g. treats it as a percent).
    steer_gain: float = 1.0
    throttle_delay_s: float = 0.0          # actuator pipeline delay
    steer_delay_s: float = 0.0
    slip_angle_at_v: float = 0.0           # coarse tire-slip correction
    # --- tire grip (calibrated from bag 231452) ----------------------------
    # Real karts can't sustain unlimited a_lat — above this the tires slip
    # (kart understeers) and speed bleeds toward sqrt(grip * L / |tan(delta)|).
    # The bag shows the kart steady-states at v ≈ sqrt(grip/kappa) in tight
    # turns: at kappa=0.14, v_steady ≈ 4.38 m/s → grip ≈ v² * kappa ≈ 2.7.
    # NOTE: this is DIFFERENT from a_lat_max_mps2 above, which is the planner's
    # cost-shaping bound (typically more optimistic than reality).
    tire_a_lat_max_mps2: float = 2.7
    tire_slip_tau_s: float = 0.2           # how fast slip bleeds excess speed

    def to_bicycle_params(self):
        """Convert to the BicycleParams dataclass the DataSim runtime uses."""
        from sim.identify_bicycle import BicycleParams
        # Map only the fields BicycleParams accepts.
        fields = {
            "accel_tau", "brake_tau", "throttle_delay_s", "steer_delay_s",
            "steer_gain", "steer_slop_deg", "wheelbase_m",
            "steer_rate_max_degps", "slip_angle_at_v", "v_max_mps",
            "tire_a_lat_max_mps2", "tire_slip_tau_s",
        }
        return BicycleParams(**{k: v for k, v in asdict(self).items() if k in fields})

    def to_kart_constants(self):
        """Convert to the KartConstants the MPC planner reads."""
        from autonomous_kart.nodes.pathfinder.planners.base import KartConstants
        return KartConstants(
            v_max_mps=self.v_max_mps,
            wheelbase_m=self.wheelbase_m,
            steer_max_deg=self.steer_max_deg,
            steer_rate_max_degps=self.steer_rate_max_degps,
            a_max_mps2=self.a_max_mps2,
            a_min_mps2=self.a_min_mps2,
            a_lat_max_mps2=self.a_lat_max_mps2,
        )

    def as_dict(self) -> dict:
        return asdict(self)
