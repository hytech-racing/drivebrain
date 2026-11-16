"""Rerun time-series panels + blueprint. The 3D scene (cones, car, lidar) lives
in sim/render.py."""
import rerun as rr
import rerun.blueprint as rrb


def blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(name="world", origin="/world"),
            rrb.Vertical(
                rrb.TimeSeriesView(name="torques [Nm]", contents=["scalars/torque_*"]),
                rrb.TimeSeriesView(name="steering [rad]", contents=["scalars/steering"]),
                rrb.TimeSeriesView(name="speed [m/s]", contents=["scalars/speed"]),
                rrb.TimeSeriesView(name="yaw rate [r/s]", contents=["scalars/yaw_rate"]),
                rrb.TimeSeriesView(name="slip [rad]", contents=["scalars/slip"]),
            ),
            column_shares=[2, 1],
        )
    )


def setup_series() -> None:
    rr.log("scalars/torque_fl", rr.SeriesLines(colors=[[255, 80, 80]], names=["FL"]), static=True)
    rr.log("scalars/torque_fr", rr.SeriesLines(colors=[[80, 255, 80]], names=["FR"]), static=True)
    rr.log("scalars/torque_rl", rr.SeriesLines(colors=[[80, 80, 255]], names=["RL"]), static=True)
    rr.log("scalars/torque_rr", rr.SeriesLines(colors=[[255, 200, 0]], names=["RR"]), static=True)
    rr.log("scalars/steering", rr.SeriesLines(colors=[[200, 200, 200]], names=["steer"]), static=True)
    rr.log("scalars/speed", rr.SeriesLines(colors=[[0, 220, 220]], names=["speed"]), static=True)
    rr.log("scalars/yaw_rate", rr.SeriesLines(colors=[[220, 100, 220]], names=["yaw_rate"]), static=True)
    rr.log("scalars/slip", rr.SeriesLines(colors=[[255, 140, 0]], names=["slip"]), static=True)


def log_scalars(
    *,
    speed: float,
    yaw_rate: float,
    slip: float,
    torque_fl: float, torque_fr: float,
    torque_rl: float, torque_rr: float,
    steering: float,
) -> None:
    """Log one sample per series at the current sim time (set by the caller)."""
    rr.log("scalars/torque_fl", rr.Scalars(torque_fl))
    rr.log("scalars/torque_fr", rr.Scalars(torque_fr))
    rr.log("scalars/torque_rl", rr.Scalars(torque_rl))
    rr.log("scalars/torque_rr", rr.Scalars(torque_rr))
    rr.log("scalars/steering", rr.Scalars(steering))
    rr.log("scalars/speed", rr.Scalars(speed))
    rr.log("scalars/yaw_rate", rr.Scalars(yaw_rate))
    rr.log("scalars/slip", rr.Scalars(slip))
