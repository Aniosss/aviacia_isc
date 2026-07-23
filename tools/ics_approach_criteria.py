from __future__ import annotations

import math
from dataclasses import dataclass

from ics_protocol import ICSInputs


def angular_error_deg(reference_deg: float, actual_deg: float) -> float:
    return (actual_deg - reference_deg + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class ApproachCriteriaConfig:
    cutoff_radio_altitude_ft: float = 300.0
    max_course_error_deg: float = 0.7
    max_glideslope_error_deg: float = 0.5
    target_glideslope_deg: float = 3.0


@dataclass(frozen=True)
class ApproachCriteriaSample:
    course_error_deg: float | None
    glideslope_error_deg: float | None
    status: str


@dataclass(frozen=True)
class ApproachCriteriaVerdict:
    status: str
    sample_count: int
    start_radio_altitude_ft: float | None
    end_radio_altitude_ft: float | None
    max_course_error_deg: float | None
    max_glideslope_error_deg: float | None
    reasons: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return self.status == "PASS"

    def summary(self) -> str:
        course = (
            "n/a" if self.max_course_error_deg is None
            else f"{self.max_course_error_deg:.3f}deg"
        )
        glide = (
            "n/a" if self.max_glideslope_error_deg is None
            else f"{self.max_glideslope_error_deg:.3f}deg"
        )
        details = "; ".join(self.reasons) if self.reasons else "all limits satisfied"
        return (
            f"CRITERIA {self.status}: samples={self.sample_count} "
            f"max_course={course} max_glideslope={glide}; {details}"
        )


class ApproachCriteriaMonitor:
    """Evaluate the A.1.1 approach segment and stop at a radio-altitude floor."""

    def __init__(self, config: ApproachCriteriaConfig | None = None) -> None:
        self.config = config or ApproachCriteriaConfig()
        self.sample_count = 0
        self.start_radio_altitude_ft: float | None = None
        self.end_radio_altitude_ft: float | None = None
        self.max_course_error_deg = 0.0
        self.max_glideslope_error_deg = 0.0
        self.invalid_reasons: set[str] = set()
        self.cutoff_reached = False

    def observe(
        self,
        state: ICSInputs,
        flight_path_angle_deg: float,
    ) -> ApproachCriteriaSample:
        if self.cutoff_reached:
            return ApproachCriteriaSample(None, None, "COMPLETE")

        if not math.isfinite(state.RadioAltitude):
            self.invalid_reasons.add("radio altitude is non-finite")
            return ApproachCriteriaSample(None, None, "INVALID")
        if state.RadioAltitude <= self.config.cutoff_radio_altitude_ft:
            self.cutoff_reached = True
            self.end_radio_altitude_ft = state.RadioAltitude
            return ApproachCriteriaSample(None, None, "COMPLETE")

        if self.start_radio_altitude_ft is None:
            self.start_radio_altitude_ft = state.RadioAltitude
        self.end_radio_altitude_ft = state.RadioAltitude

        missing = []
        if not math.isfinite(state.RunwayHeading):
            missing.append("runway heading non-finite")
        if not math.isfinite(state.TrkAngleMagnetic):
            missing.append("magnetic track non-finite")
        if not math.isfinite(flight_path_angle_deg):
            missing.append("flight-path angle non-finite")
        if missing:
            self.invalid_reasons.update(missing)
            return ApproachCriteriaSample(None, None, "INVALID")

        course_error = abs(
            angular_error_deg(state.RunwayHeading, state.TrkAngleMagnetic)
        )
        glideslope_error = abs(
            flight_path_angle_deg + self.config.target_glideslope_deg
        )
        # Do not judge the initial ILS intercept transient. Monitoring begins
        # once course and glideslope are simultaneously inside their limits;
        # every subsequent sample remains part of the acceptance result.
        if self.sample_count == 0 and (
            course_error > self.config.max_course_error_deg
            or glideslope_error > self.config.max_glideslope_error_deg
        ):
            return ApproachCriteriaSample(
                course_error,
                glideslope_error,
                "WAITING_CAPTURE",
            )

        self.sample_count += 1
        self.max_course_error_deg = max(self.max_course_error_deg, course_error)
        self.max_glideslope_error_deg = max(
            self.max_glideslope_error_deg,
            glideslope_error,
        )
        status = "OK"
        if (
            course_error > self.config.max_course_error_deg
            or glideslope_error > self.config.max_glideslope_error_deg
        ):
            status = "LIMIT"
        return ApproachCriteriaSample(course_error, glideslope_error, status)

    def verdict(self) -> ApproachCriteriaVerdict:
        reasons = sorted(self.invalid_reasons)
        if not self.cutoff_reached:
            status = "INCOMPLETE"
            reasons.append(
                "300-ft cutoff was not reached"
                if self.config.cutoff_radio_altitude_ft == 300.0
                else f"{self.config.cutoff_radio_altitude_ft:g}-ft cutoff was not reached"
            )
        elif self.sample_count == 0:
            status = "FAIL"
            reasons.append("criteria were not captured above the cutoff")
        else:
            if self.max_course_error_deg > self.config.max_course_error_deg:
                reasons.append(
                    f"course {self.max_course_error_deg:.3f}deg exceeds "
                    f"{self.config.max_course_error_deg:g}deg"
                )
            if self.max_glideslope_error_deg > self.config.max_glideslope_error_deg:
                reasons.append(
                    f"glideslope {self.max_glideslope_error_deg:.3f}deg exceeds "
                    f"{self.config.max_glideslope_error_deg:g}deg"
                )
            status = "PASS" if not reasons else "FAIL"
        return ApproachCriteriaVerdict(
            status=status,
            sample_count=self.sample_count,
            start_radio_altitude_ft=self.start_radio_altitude_ft,
            end_radio_altitude_ft=self.end_radio_altitude_ft,
            max_course_error_deg=(
                self.max_course_error_deg if self.sample_count else None
            ),
            max_glideslope_error_deg=(
                self.max_glideslope_error_deg if self.sample_count else None
            ),
            reasons=tuple(reasons),
        )
