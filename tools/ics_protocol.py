from __future__ import annotations

import json
from dataclasses import asdict, dataclass, fields
from enum import IntEnum
from typing import Any, Optional


CONTROL_COMMAND_COUNT = 14
ALL_CONTROL_VALID_MASK = (1 << CONTROL_COMMAND_COUNT) - 1
AIRBORNE_CONTROL_VALID_MASK = (1 << 5) - 1


class GearState(IntEnum):
    NoneState = 0
    UpLock = 1
    Move = 2
    DownLock = 3


class ControlModeState(IntEnum):
    Off = 0
    Approach = 1
    Landing = 2
    Rollout = 3
    Taxi = 4
    ManualTest = 5


class ReverseEngineType(IntEnum):
    Off = 0
    Arm = 1
    Deploy = 2


@dataclass(frozen=True)
class ICSInputs:
    AgentIsActive: int
    FlightPhaseValid: int
    FlightPhase: int
    LatitudeValid: int
    Latitude: float
    LongitudeValid: int
    Longitude: float
    RadioAltitudeValid: int
    RadioAltitude: float
    BaroAltitudeValid: int
    BaroAltitude: float
    IndicatedAirspeedValid: int
    IndicatedAirspeed: float
    TrueAirspeedValid: int
    TrueAirspeed: float
    GroundSpeedValid: int
    GroundSpeed: float
    VerticalSpeedValid: int
    VerticalSpeed: float
    PitchAngleValid: int
    PitchAngle: float
    RollAngleValid: int
    RollAngle: float
    MagneticHeadingValid: int
    MagneticHeading: float
    TrueHeadingValid: int
    TrueHeading: float
    TrkAngleMagneticValid: int
    TrkAngleMagnetic: float
    TrkAngleTrueValid: int
    TrkAngleTrue: float
    BodyPitchRateValid: int
    BodyPitchRate: float
    BodyRollRateValid: int
    BodyRollRate: float
    BodyYawRateValid: int
    BodyYawRate: float
    BodyNormAccelValid: int
    BodyNormAccel: float
    BodyLongAccelValid: int
    BodyLongAccel: float
    BodyLatAccelValid: int
    BodyLatAccel: float
    RunwayHeadingValid: int
    RunwayHeading: float
    RunwayLength: float
    RunwayWidth: float
    LateralDeviation: float
    LocDeviationValid: int
    LocDeviation: float
    GSDeviationValid: int
    GSDeviation: float
    NoseGearStatus: GearState
    LeftGearStatus: GearState
    RightGearStatus: GearState
    NoseGearWeightOnWheels: int
    LeftGearWeightOnWheels: int
    RightGearWeightOnWheels: int
    NoseWheelAngle: float
    SlatsAngle: float
    FlapsAngle: float
    StabilizerAngle: float
    ElevatorLeftAngle: float
    ElevatorRightAngle: float
    AileronLeftAngle: float
    AileronRightAngle: float
    RudderAngle: float
    EngLeftThrust: float
    EngRigntThrust: float
    LeftThrottleAngle: float
    RightThrottleAngle: float
    LeftBrakePedal: float
    RightBrakePedal: float
    LeftSpoiler1: float
    LeftSpoiler2: float
    LeftSpoiler3: float
    LeftSpoiler4: float
    LeftAirBrake: float
    RightSpoiler1: float
    RightSpoiler2: float
    RightSpoiler3: float
    RightSpoiler4: float
    RightAirBrake: float
    WindDirectionTrue: float
    WindSpeed: float
    Visibility: float
    PrecipitationRatio: float
    RunwayCondition: int
    AirfieldTemp: float
    FaultLeftEngine: int
    FaultRightEngine: int
    FaultLeftEngineReverse: int
    FaultRightEngineReverse: int
    FaultLeftLandingGear: int
    FaultRightLandingGear: int
    FaultNoseLandingGear: int
    FaultLeftStab: int
    FaultRightStab: int
    FaultNWS: int

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ICSInputs:
        field_names = {field.name for field in fields(cls)}
        normalized = {key: value for key, value in data.items() if key in field_names}
        normalized["NoseGearStatus"] = GearState(normalized["NoseGearStatus"])
        normalized["LeftGearStatus"] = GearState(normalized["LeftGearStatus"])
        normalized["RightGearStatus"] = GearState(normalized["RightGearStatus"])
        return cls(**normalized)

    @classmethod
    def from_json_bytes(cls, data: bytes) -> ICSInputs:
        decoded = json.loads(data.decode("utf-8"))
        if not isinstance(decoded, dict):
            raise ValueError(f"Expected JSON object, got {type(decoded).__name__}")
        return cls.from_dict(decoded)


@dataclass
class ICSOutputs:
    ControlValidMask: int = 0
    ControlMode: ControlModeState = ControlModeState.Off
    ElevatorCmd: float = 0.0
    AileronCmd: float = 0.0
    RudderCmd: float = 0.0
    ThrottleLeftRate: float = 0.0
    ThrottleRightRate: float = 0.0
    ThrottleLeft: float = 0.0
    ThrottleRight: float = 0.0
    NoseWheelTillerCmd: float = 0.0
    RudderPedalCmd: float = 0.0
    BrakeLeftCmd: float = 0.0
    BrakeRightCmd: float = 0.0
    AirbrakeCmd: float = 0.0
    ReverseLeftCmd: ReverseEngineType = ReverseEngineType.Off
    ReverseRightCmd: ReverseEngineType = ReverseEngineType.Off
    ModeAIReady: int = 0
    ModeLocCapture: int = 0
    ModeLocTrack: int = 0
    ModeGSCapture: int = 0
    ModeGSTrack: int = 0
    ModeFlareArm: int = 0
    ModeFlare: int = 0
    ModeAlignArm: int = 0
    ModeAlign: int = 0
    ModeRolloutArm: int = 0
    ModeRollout: int = 0
    ModeTaxiArm: int = 0
    ModeTaxi: int = 0
    ModeSpeed: int = 0
    ModeThrust: int = 0
    WarningFlags: int = 0
    QualityLateralError: float = 0.0
    QualityHeadingError: float = 0.0
    QualitySpeedError: float = 0.0
    reserved: Optional[str] = None

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["ControlMode"] = int(self.ControlMode)
        data["ReverseLeftCmd"] = int(self.ReverseLeftCmd)
        data["ReverseRightCmd"] = int(self.ReverseRightCmd)
        return data

    def to_json_bytes(self) -> bytes:
        return json.dumps(self.to_dict(), separators=(",", ":")).encode("utf-8")
