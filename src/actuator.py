from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PublicActuatorEnvelope:
    name: str
    rated_voltage_v: float
    voltage_range_v: tuple[float, float]
    no_load_speed_rpm: float
    rotating_rated_torque_nm: float
    rotating_rated_speed_rpm: float
    stall_continuous_torque_nm: float
    peak_torque_nm: float
    mass_kg: float
    peak_curve_rpm_nm: tuple[tuple[float, float], ...]
    rotating_overload_s: tuple[tuple[float, float], ...]
    stall_overload_s: tuple[tuple[float, float], ...]

    def peak_torque_at_speed(self, speed_rad_s: float) -> float:
        """Return the approximate 48 V peak torque curve digitized from the manual."""
        speed_rpm = abs(speed_rad_s) * 60.0 / (2.0 * np.pi)
        points = np.asarray(self.peak_curve_rpm_nm, dtype=float)
        return float(
            np.interp(
                speed_rpm,
                points[:, 0],
                points[:, 1],
                left=points[0, 1],
                right=0.0,
            )
        )

    def conservative_overload_time_s(
        self, torque_nm: float, *, stalled: bool
    ) -> float:
        """Return a table-backed conservative duration without interpolating claims."""
        torque = abs(torque_nm)
        continuous = (
            self.stall_continuous_torque_nm
            if stalled
            else self.rotating_rated_torque_nm
        )
        if torque <= continuous:
            return np.inf
        table = self.stall_overload_s if stalled else self.rotating_overload_s
        for load_nm, time_s in table:
            if torque <= load_nm:
                return time_s
        return 0.0


EL05_PUBLIC_ENVELOPE = PublicActuatorEnvelope(
    name="RobStride EduLite EL05",
    rated_voltage_v=48.0,
    voltage_range_v=(15.0, 60.0),
    no_load_speed_rpm=430.0,
    rotating_rated_torque_nm=1.8,
    rotating_rated_speed_rpm=100.0,
    stall_continuous_torque_nm=1.1,
    peak_torque_nm=6.0,
    mass_kg=0.242,
    # Approximate visual digitization of the manufacturer's 48 V T-N plot.
    # The final (430, 0) point closes the plotted curve at the separately stated
    # no-load speed; it is an inference, not an additional manufacturer datum.
    peak_curve_rpm_nm=(
        (0.0, 6.0),
        (30.0, 6.0),
        (100.0, 5.55),
        (150.0, 5.25),
        (200.0, 4.90),
        (240.0, 4.45),
        (270.0, 3.95),
        (300.0, 3.20),
        (330.0, 2.65),
        (360.0, 2.15),
        (380.0, 1.60),
        (405.0, 1.00),
        (430.0, 0.0),
    ),
    rotating_overload_s=(
        (2.0, 300.0),
        (3.0, 44.0),
        (4.0, 14.0),
        (5.0, 7.0),
        (6.0, 5.0),
    ),
    stall_overload_s=(
        (1.8, 175.0),
        (3.0, 13.0),
        (4.0, 6.0),
        (5.0, 2.0),
        (6.0, 1.0),
    ),
)
