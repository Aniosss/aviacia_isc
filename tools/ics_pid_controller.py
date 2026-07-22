from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from ics_flight_envelope import (
    LandingFlapConfiguration,
    approach_limits,
    detect_landing_flaps,
    roll_limit_deg,
)
from ics_protocol import ICSInputs


def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def angle_error_deg(target: float, actual: float) -> float:
    return (target - actual + 180.0) % 360.0 - 180.0


@dataclass
class PIDConfig:
    kp: float
    ki: float
    kd: float
    output_min: float
    output_max: float
    integrator_min: float
    integrator_max: float
    derivative_tau_s: float = 0.25


class PID:
    """Filtered PID with conditional-integration anti-windup."""

    def __init__(self, config: PIDConfig) -> None:
        self.config = config
        self.integral = 0.0
        self.derivative = 0.0
        self._previous_measurement: float | None = None

    def reset(self) -> None:
        self.integral = 0.0
        self.derivative = 0.0
        self._previous_measurement = None

    def update(
        self,
        error: float,
        measurement: float,
        dt_s: float,
        output_min: float | None = None,
        output_max: float | None = None,
    ) -> float:
        dt = clamp(dt_s, 0.001, 0.25)
        active_output_min = self.config.output_min if output_min is None else output_min
        active_output_max = self.config.output_max if output_max is None else output_max
        if active_output_min > active_output_max:
            raise ValueError("output_min must not exceed output_max")
        if self._previous_measurement is None:
            raw_derivative = 0.0
        else:
            raw_derivative = -(measurement - self._previous_measurement) / dt
        alpha = 1.0 if self.config.derivative_tau_s == 0.0 else -math.expm1(
            -dt / self.config.derivative_tau_s
        )
        self.derivative += alpha * (raw_derivative - self.derivative)

        candidate_integral = clamp(
            self.integral + error * dt,
            self.config.integrator_min,
            self.config.integrator_max,
        )
        candidate = (
            self.config.kp * error
            + self.config.ki * candidate_integral
            + self.config.kd * self.derivative
        )
        output = clamp(candidate, active_output_min, active_output_max)
        integral_delta = self.config.ki * (candidate_integral - self.integral)
        can_integrate = (
            active_output_min <= candidate <= active_output_max
            or (candidate > active_output_max and integral_delta < 0.0)
            or (candidate < active_output_min and integral_delta > 0.0)
        )
        if can_integrate:
            self.integral = candidate_integral
        self._previous_measurement = measurement
        return output


@dataclass
class ControllerConfig:
    loc_full_scale_ddm: float = 0.155
    gs_full_scale_ddm: float = 0.175
    localizer_sign: float = 1.0
    localizer_intercept_deg_per_dot: float = 10.0
    max_intercept_angle_deg: float = 15.0
    heading_to_roll_gain: float = 0.70
    max_roll_target_deg: float = 15.0
    glideslope_sign: float = -1.0
    glideslope_vs_correction_fpm_per_dot: float = 400.0
    glideslope_angle_deg: float = 3.0
    approach_aoa_deg: float = 7.1
    adaptive_aoa_enabled: bool = True
    adaptive_aoa_filter_tau_s: float = 10.0
    adaptive_aoa_min_deg: float = 2.0
    adaptive_aoa_max_deg: float = 10.0
    adaptive_aoa_max_gs_dots: float = 0.25
    adaptive_aoa_max_vs_error_fpm: float = 250.0
    adaptive_aoa_rate_deg_per_s: float = 0.2
    adaptive_aoa_recovery_rate_deg_per_s: float = 0.08
    vs_to_pitch_gain_deg_per_fpm: float = 0.001
    vs_to_pitch_fast_descent_gain_deg_per_fpm: float = 0.0015
    min_approach_target_vs_fpm: float = -900.0
    max_approach_target_vs_fpm: float = -300.0
    min_vertical_correction_deg: float = -1.0
    max_vertical_correction_deg: float = 2.0
    pitch_target_rate_deg_per_s: float = 1.5
    min_pitch_target_deg: float = -1.0
    max_pitch_target_deg: float = 8.0
    landing_weight_kg: float = 69277.0
    landing_flap_fallback: str = "FLAPS_3"
    flare_arm_radio_altitude_ft: float = 400.0
    flare_start_radio_altitude_ft: float = 150.0
    flare_max_start_radio_altitude_ft: float = 200.0
    flare_time_to_ground_s: float = 15.0
    flare_end_radio_altitude_ft: float = 5.0
    flare_initial_vs_fpm: float = -472.44
    touchdown_target_vs_fpm: float = -68.90
    flare_vs_to_pitch_gain_deg_per_fpm: float = 0.00635
    flare_pitch_base_deg: float = 1.8
    flare_pitch_attitude_damping_gain: float = 0.15
    flare_pitch_rate_damping_gain: float = 0.08
    flare_min_pitch_target_deg: float = 0.5
    flare_pitch_target_rate_deg_per_s: float = 4.0
    flare_max_pitch_target_deg: float = 6.0
    elevator_command_sign: float = 1.0
    throttle_forward_max_deg: float = 55.7
    throttle_rate_max_deg_per_s: float = 8.0
    throttle_left_rate_sign: float = 1.0
    throttle_right_rate_sign: float = 1.0
    throttle_position_gain_per_s: float = 0.8
    throttle_sync_boost_threshold_deg: float = 1.0
    throttle_sync_boost_gain_per_s: float = 2.0
    target_ias_rate_kt_per_s: float = 0.25
    roll_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(-1.0, 0.0, -0.25, -10.0, 10.0, -40.0, 40.0, 0.20)
    )
    pitch_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(0.20, 0.0045, 0.06, -0.5, 0.5, -20.0, 20.0, 0.20)
    )
    speed_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(
            0.005,
            0.0,
            0.005,
            -0.10,
            0.10,
            -100.0,
            100.0,
            0.50,
        )
    )

    @classmethod
    def from_json(cls, path: str | Path) -> ControllerConfig:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        cfg = cls()
        for key, value in raw.items():
            if key in {"roll_pid", "pitch_pid", "speed_pid"}:
                setattr(cfg, key, PIDConfig(**value))
            elif hasattr(cfg, key):
                setattr(cfg, key, value)
        return cfg


@dataclass
class ControlResult:
    aileron: float
    elevator: float
    rudder: float
    throttle_left_rate: float
    throttle_right_rate: float
    throttle_norm: float
    throttle_target_angle_deg: float
    throttle_left_hold_norm: float
    throttle_right_hold_norm: float
    loc_dots: float
    gs_dots: float
    target_heading_deg: float
    heading_error_deg: float
    target_roll_deg: float
    target_vs_fpm: float
    target_pitch_deg: float
    vertical_correction_deg: float
    flight_path_angle_deg: float
    target_flight_path_angle_deg: float
    estimated_aoa_deg: float
    reference_aoa_deg: float
    mach: float
    target_ias_kt: float
    vapp_kt: float
    flap_configuration: str
    table_weight_kg: float
    vsr1_kt: float
    vfe_kt: float
    alpha_prot_deg: float
    alpha_sw_deg: float
    alpha_margin_deg: float
    roll_limit_deg: float
    flare_armed: bool
    flare_active: bool
    flare_progress: float
    flare_entry_radio_altitude_ft: float
    flare_entry_vertical_speed_fpm: float
    touchdown_vertical_speed_limit_fpm: float
    touchdown_speed_min_kt: float
    touchdown_speed_max_kt: float
    touchdown_pitch_limit_deg: float
    envelope_warnings: tuple[str, ...]


class ClearWeatherILSController:
    def __init__(self, config: ControllerConfig) -> None:
        self.config = config
        self.roll_pid = PID(config.roll_pid)
        self.pitch_pid = PID(config.pitch_pid)
        self.speed_pid = PID(config.speed_pid)
        self._target_pitch_deg: float | None = None
        self._reference_aoa_deg: float | None = None
        self._flare_active = False
        self._flare_entry_radio_altitude_ft: float | None = None
        self._flare_entry_vertical_speed_fpm: float | None = None
        self._flare_entry_pitch_target_deg: float | None = None
        self._throttle_norm: float | None = None
        self._target_ias_kt: float | None = None

    def reset(self) -> None:
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.speed_pid.reset()
        self._target_pitch_deg = None
        self._reference_aoa_deg = None
        self._flare_active = False
        self._flare_entry_radio_altitude_ft = None
        self._flare_entry_vertical_speed_fpm = None
        self._flare_entry_pitch_target_deg = None
        self._throttle_norm = None
        self._target_ias_kt = None

    def update(self, state: ICSInputs, dt_s: float) -> ControlResult:
        cfg = self.config
        fallback_flaps = LandingFlapConfiguration(cfg.landing_flap_fallback)
        flap_configuration = detect_landing_flaps(state.FlapsAngle, fallback_flaps)
        static_temperature_k = max(180.0, 273.15 + state.AirfieldTemp)
        speed_of_sound_kt = 38.967854 * math.sqrt(static_temperature_k)
        mach = (
            state.TrueAirspeed / speed_of_sound_kt
            if state.TrueAirspeedValid and state.TrueAirspeed > 0.0
            else 0.32
        )
        limits = approach_limits(cfg.landing_weight_kg, flap_configuration, mach)
        if self._target_ias_kt is None:
            self._target_ias_kt = (
                state.IndicatedAirspeed
                if state.IndicatedAirspeedValid and state.IndicatedAirspeed > 0.0
                else limits.vapp_kt
            )
        speed_target_step = cfg.target_ias_rate_kt_per_s * clamp(dt_s, 0.001, 0.25)
        self._target_ias_kt += clamp(
            limits.vapp_kt - self._target_ias_kt,
            -speed_target_step,
            speed_target_step,
        )
        target_ias = self._target_ias_kt
        loc_dots = state.LocDeviation / cfg.loc_full_scale_ddm
        gs_dots = cfg.glideslope_sign * state.GSDeviation / cfg.gs_full_scale_ddm

        intercept = clamp(
            cfg.localizer_sign * cfg.localizer_intercept_deg_per_dot * loc_dots,
            -cfg.max_intercept_angle_deg,
            cfg.max_intercept_angle_deg,
        )
        target_heading = (state.RunwayHeading + intercept) % 360.0
        heading_error = angle_error_deg(target_heading, state.MagneticHeading)
        active_roll_limit = roll_limit_deg(state.RadioAltitude, cfg.max_roll_target_deg)
        target_roll = clamp(
            cfg.heading_to_roll_gain * heading_error,
            -active_roll_limit,
            active_roll_limit,
        )
        roll_error = target_roll - state.RollAngle
        aileron = self.roll_pid.update(roll_error, state.RollAngle, dt_s)

        groundspeed_fpm = max(state.GroundSpeed, 35.0) * 101.268591
        nominal_vs = -groundspeed_fpm * math.tan(math.radians(cfg.glideslope_angle_deg))
        # On this simulator positive normalized guidance means below the glidepath.
        target_vs = clamp(
            nominal_vs + cfg.glideslope_vs_correction_fpm_per_dot * gs_dots,
            cfg.min_approach_target_vs_fpm,
            cfg.max_approach_target_vs_fpm,
        )
        flare_armed = bool(state.RadioAltitude <= cfg.flare_arm_radio_altitude_ft)
        if not self._flare_active:
            fixed_height_trigger = state.RadioAltitude <= cfg.flare_start_radio_altitude_ft
            descent_rate_fps = max(-state.VerticalSpeed / 60.0, 0.0)
            time_to_ground_s = (
                state.RadioAltitude / descent_rate_fps
                if descent_rate_fps > 0.1
                else math.inf
            )
            time_trigger = (
                state.VerticalSpeed < -50.0
                and state.RadioAltitude <= cfg.flare_max_start_radio_altitude_ft
                and time_to_ground_s <= cfg.flare_time_to_ground_s
            )
            if fixed_height_trigger or time_trigger:
                self._flare_active = True
                self._flare_entry_radio_altitude_ft = clamp(
                    max(state.RadioAltitude, cfg.flare_start_radio_altitude_ft),
                    cfg.flare_end_radio_altitude_ft + 1.0,
                    cfg.flare_max_start_radio_altitude_ft,
                )
                # The accepted upstream baseline starts roundout from a fixed
                # vertical-speed reference. The pitch target remains slew
                # limited, so this does not create an elevator step at entry.
                self._flare_entry_vertical_speed_fpm = cfg.flare_initial_vs_fpm
                self._flare_entry_pitch_target_deg = clamp(
                    self._target_pitch_deg
                    if self._target_pitch_deg is not None
                    else state.PitchAngle,
                    cfg.flare_min_pitch_target_deg,
                    cfg.flare_max_pitch_target_deg,
                )

        flare_active = self._flare_active
        flare_progress = 0.0
        if flare_active:
            flare_entry_altitude = (
                self._flare_entry_radio_altitude_ft
                if self._flare_entry_radio_altitude_ft is not None
                else cfg.flare_start_radio_altitude_ft
            )
            flare_entry_vs = (
                self._flare_entry_vertical_speed_fpm
                if self._flare_entry_vertical_speed_fpm is not None
                else cfg.flare_initial_vs_fpm
            )
            flare_span = max(flare_entry_altitude - cfg.flare_end_radio_altitude_ft, 1.0)
            linear_progress = (
                flare_entry_altitude - state.RadioAltitude
            ) / flare_span
            flare_progress = clamp(linear_progress, 0.0, 1.0)
            target_vs = flare_entry_vs + flare_progress * (
                cfg.touchdown_target_vs_fpm - flare_entry_vs
            )
        flight_path_angle = math.degrees(math.atan2(state.VerticalSpeed, groundspeed_fpm))
        target_flight_path_angle = math.degrees(math.atan2(target_vs, groundspeed_fpm))
        estimated_aoa = state.PitchAngle - flight_path_angle
        vertical_speed_error = target_vs - state.VerticalSpeed
        aoa_measurement_valid = bool(
            state.PitchAngleValid and state.VerticalSpeedValid and state.GroundSpeedValid
        )
        if cfg.adaptive_aoa_enabled and aoa_measurement_valid:
            measured_reference_aoa = clamp(
                estimated_aoa,
                cfg.adaptive_aoa_min_deg,
                min(cfg.adaptive_aoa_max_deg, limits.alpha_prot_deg - 0.5),
            )
            configured_reference_aoa = clamp(
                cfg.approach_aoa_deg,
                cfg.adaptive_aoa_min_deg,
                min(cfg.adaptive_aoa_max_deg, limits.alpha_prot_deg - 0.5),
            )
            aoa_learning_allowed = bool(
                not flare_active
                and abs(gs_dots) <= cfg.adaptive_aoa_max_gs_dots
                and abs(vertical_speed_error) <= cfg.adaptive_aoa_max_vs_error_fpm
            )
            if self._reference_aoa_deg is None:
                self._reference_aoa_deg = measured_reference_aoa
            elif aoa_learning_allowed:
                aoa_dt = clamp(dt_s, 0.001, 0.25)
                filter_alpha = (
                    1.0
                    if cfg.adaptive_aoa_filter_tau_s <= 0.0
                    else -math.expm1(-aoa_dt / cfg.adaptive_aoa_filter_tau_s)
                )
                filtered_step = filter_alpha * (
                    measured_reference_aoa - self._reference_aoa_deg
                )
                max_step = cfg.adaptive_aoa_rate_deg_per_s * aoa_dt
                self._reference_aoa_deg += clamp(filtered_step, -max_step, max_step)
            elif not flare_active and gs_dots > 0.0 and vertical_speed_error > 0.0:
                recovery_dt = clamp(dt_s, 0.001, 0.25)
                recovery_step = (
                    cfg.adaptive_aoa_recovery_rate_deg_per_s * recovery_dt
                )
                self._reference_aoa_deg += clamp(
                    configured_reference_aoa - self._reference_aoa_deg,
                    -recovery_step,
                    recovery_step,
                )
        reference_aoa = (
            self._reference_aoa_deg
            if self._reference_aoa_deg is not None
            else cfg.approach_aoa_deg
        )
        if not flare_active:
            vertical_gain = (
                cfg.vs_to_pitch_fast_descent_gain_deg_per_fpm
                if vertical_speed_error > 0.0
                else cfg.vs_to_pitch_gain_deg_per_fpm
            )
            min_vertical_correction = cfg.min_vertical_correction_deg
            max_vertical_correction = cfg.max_vertical_correction_deg
            vertical_correction = clamp(
                vertical_gain * vertical_speed_error,
                min_vertical_correction,
                max_vertical_correction,
            )
            raw_target_pitch = clamp(
                reference_aoa + target_flight_path_angle + vertical_correction,
                cfg.min_pitch_target_deg,
                cfg.max_pitch_target_deg,
            )
        else:
            # Port of the accepted upstream flare_vs_hold law. Unlike the
            # approach AoA law, roundout follows vertical-speed error directly
            # and damps both attitude and pitch rate.
            vertical_correction = cfg.flare_vs_to_pitch_gain_deg_per_fpm * vertical_speed_error
            pitch_rate = state.BodyPitchRate if state.BodyPitchRateValid else 0.0
            raw_target_pitch = clamp(
                cfg.flare_pitch_base_deg
                + vertical_correction
                - cfg.flare_pitch_attitude_damping_gain * state.PitchAngle
                - cfg.flare_pitch_rate_damping_gain * pitch_rate,
                cfg.flare_min_pitch_target_deg,
                cfg.flare_max_pitch_target_deg,
            )
            if self._flare_entry_pitch_target_deg is not None:
                raw_target_pitch = max(
                    raw_target_pitch,
                    self._flare_entry_pitch_target_deg,
                )
        if self._target_pitch_deg is None:
            self._target_pitch_deg = clamp(
                state.PitchAngle,
                cfg.min_pitch_target_deg,
                cfg.max_pitch_target_deg,
            )
        pitch_target_rate = (
            cfg.flare_pitch_target_rate_deg_per_s
            if flare_active
            else cfg.pitch_target_rate_deg_per_s
        )
        target_step = pitch_target_rate * clamp(dt_s, 0.001, 0.25)
        self._target_pitch_deg += clamp(
            raw_target_pitch - self._target_pitch_deg,
            -target_step,
            target_step,
        )
        target_pitch = self._target_pitch_deg
        pitch_error = target_pitch - state.PitchAngle
        elevator_effort = self.pitch_pid.update(
            pitch_error,
            state.PitchAngle,
            dt_s,
            output_min=cfg.pitch_pid.output_min,
            output_max=cfg.pitch_pid.output_max,
        )
        if not math.isfinite(elevator_effort):
            raise ValueError("non-finite elevator command")
        elevator = cfg.elevator_command_sign * elevator_effort

        speed_error = target_ias - state.IndicatedAirspeed
        if self._throttle_norm is None:
            measured_throttle_deg = 0.5 * (
                state.LeftThrottleAngle + state.RightThrottleAngle
            )
            self._throttle_norm = clamp(
                measured_throttle_deg / cfg.throttle_forward_max_deg,
                0.0,
                1.0,
            )
        previous_speed_integral = self.speed_pid.integral
        throttle_norm_rate = self.speed_pid.update(
            speed_error,
            state.IndicatedAirspeed,
            dt_s,
        )
        throttle_step = throttle_norm_rate * clamp(dt_s, 0.001, 0.25)
        unclamped_throttle_norm = self._throttle_norm + throttle_step
        if unclamped_throttle_norm < 0.0 or unclamped_throttle_norm > 1.0:
            self.speed_pid.integral = previous_speed_integral
        self._throttle_norm = clamp(unclamped_throttle_norm, 0.0, 1.0)
        logical_throttle_rate = clamp(
            throttle_norm_rate * cfg.throttle_forward_max_deg,
            -cfg.throttle_rate_max_deg_per_s,
            cfg.throttle_rate_max_deg_per_s,
        )
        throttle_target_angle = self._throttle_norm * cfg.throttle_forward_max_deg
        throttle_position_gain = cfg.throttle_position_gain_per_s
        if (
            abs(state.LeftThrottleAngle - state.RightThrottleAngle)
            >= cfg.throttle_sync_boost_threshold_deg
        ):
            throttle_position_gain = max(
                throttle_position_gain,
                cfg.throttle_sync_boost_gain_per_s,
            )
        left_physical_rate = clamp(
            logical_throttle_rate
            + throttle_position_gain
            * (throttle_target_angle - state.LeftThrottleAngle),
            -cfg.throttle_rate_max_deg_per_s,
            cfg.throttle_rate_max_deg_per_s,
        )
        right_physical_rate = clamp(
            logical_throttle_rate
            + throttle_position_gain
            * (throttle_target_angle - state.RightThrottleAngle),
            -cfg.throttle_rate_max_deg_per_s,
            cfg.throttle_rate_max_deg_per_s,
        )
        throttle_left_rate = cfg.throttle_left_rate_sign * left_physical_rate
        throttle_right_rate = cfg.throttle_right_rate_sign * right_physical_rate
        # Some simulator builds ignore ControlValidMask. Keep both absolute
        # channels on the same target so they reinforce, rather than oppose,
        # the per-engine rate synchronizer.
        throttle_left_hold_norm = self._throttle_norm
        throttle_right_hold_norm = self._throttle_norm

        rudder = 0.0
        warnings: list[str] = []
        if state.IndicatedAirspeed >= limits.vfe_kt:
            warnings.append("VFE")
        if state.IndicatedAirspeed <= limits.vsr1_kt:
            warnings.append("VSR1")
        elif state.IndicatedAirspeed < limits.vapp_kt:
            warnings.append("BELOW_VAPP")
        if estimated_aoa >= limits.alpha_prot_deg:
            warnings.append("ALPHA_PROT")
        if estimated_aoa >= limits.alpha_sw_deg:
            warnings.append("ALPHA_SW")
        if abs(state.RollAngle) > active_roll_limit:
            warnings.append("ROLL_LIMIT")
        # IRS body-normal acceleration is unbiased: zero corresponds to steady 1 g.
        if state.BodyNormAccelValid and abs(state.BodyNormAccel) > 1.0:
            warnings.append("LOAD_FACTOR")
        if flare_active:
            if state.IndicatedAirspeed <= limits.touchdown_speed_min_kt:
                warnings.append("TOUCHDOWN_SPEED_LOW")
            elif state.IndicatedAirspeed >= limits.touchdown_speed_max_kt:
                warnings.append("TOUCHDOWN_SPEED_HIGH")
            if abs(state.VerticalSpeed) > limits.touchdown_vertical_speed_limit_fpm:
                warnings.append("TOUCHDOWN_VS")
            if not 0.0 < state.PitchAngle < limits.touchdown_pitch_limit_deg:
                warnings.append("TOUCHDOWN_PITCH")
        return ControlResult(
            aileron=aileron,
            elevator=elevator,
            rudder=rudder,
            throttle_left_rate=throttle_left_rate,
            throttle_right_rate=throttle_right_rate,
            throttle_norm=self._throttle_norm,
            throttle_target_angle_deg=throttle_target_angle,
            throttle_left_hold_norm=throttle_left_hold_norm,
            throttle_right_hold_norm=throttle_right_hold_norm,
            loc_dots=loc_dots,
            gs_dots=gs_dots,
            target_heading_deg=target_heading,
            heading_error_deg=heading_error,
            target_roll_deg=target_roll,
            target_vs_fpm=target_vs,
            target_pitch_deg=target_pitch,
            vertical_correction_deg=vertical_correction,
            flight_path_angle_deg=flight_path_angle,
            target_flight_path_angle_deg=target_flight_path_angle,
            estimated_aoa_deg=estimated_aoa,
            reference_aoa_deg=reference_aoa,
            mach=mach,
            target_ias_kt=target_ias,
            vapp_kt=limits.vapp_kt,
            flap_configuration=flap_configuration.value,
            table_weight_kg=limits.table_weight_kg,
            vsr1_kt=limits.vsr1_kt,
            vfe_kt=limits.vfe_kt,
            alpha_prot_deg=limits.alpha_prot_deg,
            alpha_sw_deg=limits.alpha_sw_deg,
            alpha_margin_deg=limits.alpha_prot_deg - estimated_aoa,
            roll_limit_deg=active_roll_limit,
            flare_armed=flare_armed,
            flare_active=flare_active,
            flare_progress=flare_progress,
            flare_entry_radio_altitude_ft=(
                self._flare_entry_radio_altitude_ft
                if self._flare_entry_radio_altitude_ft is not None
                else cfg.flare_start_radio_altitude_ft
            ),
            flare_entry_vertical_speed_fpm=(
                self._flare_entry_vertical_speed_fpm
                if self._flare_entry_vertical_speed_fpm is not None
                else state.VerticalSpeed
            ),
            touchdown_vertical_speed_limit_fpm=limits.touchdown_vertical_speed_limit_fpm,
            touchdown_speed_min_kt=limits.touchdown_speed_min_kt,
            touchdown_speed_max_kt=limits.touchdown_speed_max_kt,
            touchdown_pitch_limit_deg=limits.touchdown_pitch_limit_deg,
            envelope_warnings=tuple(warnings),
        )
