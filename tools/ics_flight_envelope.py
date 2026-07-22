from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class LandingFlapConfiguration(str, Enum):
    FLAPS_3 = "FLAPS_3"
    FULL = "FULL"


WEIGHT_ROWS_KG = (45000.0, 50000.0, 55000.0, 60000.0, 65000.0, 70000.0, 75000.0, 79250.0)

VAPP_KT = {
    LandingFlapConfiguration.FLAPS_3: (115.0, 118.0, 124.0, 129.0, 135.0, 140.0, 146.0, 150.0),
    LandingFlapConfiguration.FULL: (115.0, 115.0, 121.0, 126.0, 131.0, 136.0, 141.0, 146.0),
}

VSR1_KT = {
    LandingFlapConfiguration.FLAPS_3: (91.1, 96.0, 100.7, 105.2, 109.5, 113.6, 117.6, 121.5),
    LandingFlapConfiguration.FULL: (88.7, 93.4, 98.0, 102.3, 106.5, 110.5, 114.4, 118.2),
}

VFE_KT = {
    LandingFlapConfiguration.FLAPS_3: 183.0,
    LandingFlapConfiguration.FULL: 178.0,
}

ALPHA_SW_DEG = {
    LandingFlapConfiguration.FLAPS_3: 13.6,
    LandingFlapConfiguration.FULL: 13.2,
}

ALPHA_PROT_MACH = (0.20, 0.32, 0.40, 0.50, 0.60, 0.70, 0.75, 0.78, 0.80, 0.82, 0.85)
ALPHA_PROT_CLEAN_DEG = (7.5, 7.2, 6.7, 6.4, 6.2, 5.5, 4.8, 4.2, 3.7, 3.2, 2.2)
ALPHA_PROT_FLAP_INCREMENT_DEG = {
    LandingFlapConfiguration.FLAPS_3: 5.6,
    LandingFlapConfiguration.FULL: 5.6,
}


@dataclass(frozen=True)
class ApproachLimits:
    flap_configuration: LandingFlapConfiguration
    table_weight_kg: float
    vapp_kt: float
    vsr1_kt: float
    vfe_kt: float
    alpha_sw_deg: float
    alpha_prot_deg: float
    touchdown_speed_min_kt: float
    touchdown_speed_max_kt: float
    touchdown_vertical_speed_limit_fpm: float = 472.0
    touchdown_pitch_limit_deg: float = 8.1


def detect_landing_flaps(
    flap_angle_deg: float,
    fallback: LandingFlapConfiguration = LandingFlapConfiguration.FULL,
) -> LandingFlapConfiguration:
    measured = measured_landing_flaps(flap_angle_deg)
    return measured if measured is not None else fallback


def measured_landing_flaps(
    flap_angle_deg: float,
) -> LandingFlapConfiguration | None:
    if flap_angle_deg >= 31.5:
        return LandingFlapConfiguration.FULL
    if flap_angle_deg >= 22.0:
        return LandingFlapConfiguration.FLAPS_3
    return None


def _ceiling_weight_index(weight_kg: float) -> int:
    for index, table_weight in enumerate(WEIGHT_ROWS_KG):
        if weight_kg <= table_weight:
            return index
    return len(WEIGHT_ROWS_KG) - 1


def _linear_interpolate(x: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> float:
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for index in range(1, len(xs)):
        if x <= xs[index]:
            fraction = (x - xs[index - 1]) / (xs[index] - xs[index - 1])
            return ys[index - 1] + fraction * (ys[index] - ys[index - 1])
    raise AssertionError("interpolation interval not found")


def alpha_prot_deg(mach: float, flap_configuration: LandingFlapConfiguration) -> float:
    clean = _linear_interpolate(mach, ALPHA_PROT_MACH, ALPHA_PROT_CLEAN_DEG)
    return clean + ALPHA_PROT_FLAP_INCREMENT_DEG[flap_configuration]


def approach_limits(
    weight_kg: float,
    flap_configuration: LandingFlapConfiguration,
    mach: float = 0.32,
) -> ApproachLimits:
    index = _ceiling_weight_index(weight_kg)
    vapp = VAPP_KT[flap_configuration][index]
    return ApproachLimits(
        flap_configuration=flap_configuration,
        table_weight_kg=WEIGHT_ROWS_KG[index],
        vapp_kt=vapp,
        vsr1_kt=VSR1_KT[flap_configuration][index],
        vfe_kt=VFE_KT[flap_configuration],
        alpha_sw_deg=ALPHA_SW_DEG[flap_configuration],
        alpha_prot_deg=alpha_prot_deg(mach, flap_configuration),
        touchdown_speed_min_kt=0.96 * vapp,
        touchdown_speed_max_kt=vapp + 10.0,
    )


def roll_limit_deg(radio_altitude_ft: float, normal_limit_deg: float) -> float:
    if radio_altitude_ft <= 35.0:
        return min(normal_limit_deg, 5.0)
    if radio_altitude_ft <= 100.0:
        return min(normal_limit_deg, 10.0)
    if radio_altitude_ft <= 200.0:
        return min(normal_limit_deg, 30.0)
    return normal_limit_deg
