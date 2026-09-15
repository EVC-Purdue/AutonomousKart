import numpy as np
import pytest

from sim.plant_dataset import HISTORY, Segment
from sim.plant_reference import PLANT_HZ
from sim.plant_rollout import HORIZON_STEPS, rollout_errors


def _straight(n=400, v=5.0):
    t = np.arange(n) / PLANT_HZ
    return Segment(t=t, x=v * t, y=np.zeros(n), psi=np.zeros(n), v=np.full(n, v),
                   omega=np.zeros(n), delta_cmd=np.zeros(n),
                   throttle_sp=np.full(n, v), split="test")


def _varying_commands(n=300, v=5.0):
    # delta_cmd/throttle_sp are constant in every other fixture in this
    # file, so a stepper fed the wrong tick's command would still see
    # equal values by coincidence. Step both every tick, with different
    # scales, so any lag or channel mixup is visible.
    t = np.arange(n) / PLANT_HZ
    return Segment(t=t, x=v * t, y=np.zeros(n), psi=np.zeros(n), v=np.full(n, v),
                   omega=np.zeros(n), delta_cmd=np.arange(n, dtype=float),
                   throttle_sp=np.arange(n, dtype=float) * 2.0 + 1.0, split="test")


def test_a_perfect_stepper_has_zero_error():
    seg = _straight()
    dt = 1.0 / PLANT_HZ

    def perfect(window, delta_cmd, throttle_sp):
        return (0.0, 0.0)   # no acceleration: straight line at constant speed

    out = rollout_errors(perfect, [seg])
    for h in HORIZON_STEPS:
        assert out[h]["pos_median"] < 1e-9
        assert out[h]["n"] > 0


def test_a_biased_stepper_accumulates_linearly():
    seg = _straight()
    dt = 1.0 / PLANT_HZ

    def slow(window, delta_cmd, throttle_sp):
        return (0.0, -0.06)   # decelerating: falls behind the real kart

    # Constant deceleration a: the shortfall after n steps is the difference
    # between the real kart's constant speed and the model's ramp-down, which
    # sums to |a| * dt^2 * n(n-1)/2.
    a = -0.06
    out = rollout_errors(slow, [seg])
    for n in (10, 60):
        assert out[n]["pos_median"] == pytest.approx(
            abs(a) * dt * dt * n * (n - 1) / 2, rel=1e-6)


def test_only_test_and_val_segments_can_be_scored_together():
    # A count comparison, not just `n > 0`: with only segs[0] eligible, `n`
    # must be exactly half of what it is when both identical segments are
    # eligible. A split filter that is a silent no-op would still score
    # both segments in the "one eligible" run, making the two counts equal
    # instead of 2x -- so this fails against that bug, unlike asserting
    # `n > 0` alone, which segs[0] alone would already satisfy.
    stepper = lambda w, d, t: (0.0, 0.0)
    segs = [_straight(), _straight()]
    segs[1].split = "train"
    out_one_eligible = rollout_errors(stepper, segs, splits=("test",))

    segs[1].split = "test"
    out_both_eligible = rollout_errors(stepper, segs, splits=("test",))

    assert out_one_eligible[5]["n"] > 0
    assert out_both_eligible[5]["n"] == 2 * out_one_eligible[5]["n"]


def test_window_last_row_command_matches_the_tick_just_advanced_to():
    # The window fed to the stepper must describe the state the rollout has
    # just advanced to, command included -- not the command that was active
    # one tick earlier. Assert the invariant on every call, not just on the
    # final output, so a one-tick lag is caught even though it doesn't
    # change *this* stepper's (zero) returned deltas.
    seg = _varying_commands()

    def checking(window, delta_cmd, throttle_sp):
        assert window[-1, 2] == delta_cmd
        assert window[-1, 3] == throttle_sp
        return (0.0, 0.0)

    rollout_errors(checking, [seg])


def test_window_last_row_psi_dot_integrates_the_returned_alpha():
    # Every other fixture in this file holds the angular acceleration at 0, so
    # a bug feeding psi_dot back into the window would be invisible to them
    # (0 wrong is still 0). Return a nonzero constant alpha and check the
    # column the *next* stepper call sees, after one step folded into it.
    seg = _straight()
    dt = 1.0 / PLANT_HZ
    alpha_step = 1.2  # rad/s^2, nonzero
    calls = []

    def turning(window, delta_cmd, throttle_sp):
        calls.append(window.copy())
        return (alpha_step, 0.0)

    rollout_errors(turning, [seg], horizons=(2,))
    # calls[0] is the first call at a start point (window built directly
    # from the segment, unaffected by the step loop); calls[1] is the next
    # call at the same start point, after the first step's dpsi was folded
    # into the window.
    # seg starts at omega = 0, so after one step psi_dot is exactly alpha*dt.
    assert calls[1][-1, 1] == pytest.approx(alpha_step * dt)


def test_speed_is_clamped_the_way_the_runtime_clamps_it():
    """`LearnedPlant.step` clamps speed and the scorers did not, so the table
    graded a plant the sim does not run. Both bounds matter: without the
    ceiling a compounding speed error runs away and `is_stable`, which only
    looks at cross-track error, never sees it."""
    from sim.plant_rollout import V_MAX, V_MIN
    seg = _straight()
    seen = []

    def braking(window, delta_cmd, throttle_sp):
        seen.append(window[-1, 0])
        return (0.0, -60.0)

    rollout_errors(braking, [seg], horizons=(20,))
    assert min(seen) >= V_MIN
    assert min(seen) == pytest.approx(V_MIN)     # it did hit the floor

    seen.clear()

    def runaway(window, delta_cmd, throttle_sp):
        seen.append(window[-1, 0])
        return (0.0, 60.0)

    rollout_errors(runaway, [seg], horizons=(20,))
    assert max(seen) <= V_MAX
    assert max(seen) == pytest.approx(V_MAX)
