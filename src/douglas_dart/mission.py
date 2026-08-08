"""Phase-based, fuel-limited longitudinal mission integration."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from math import atan, cos, degrees, radians, sin

from .aero import (
    AerodynamicForces,
    BudgetAeroModel,
    LongitudinalAeroModel,
    LaunchLiftScreen,
    launch_lift_screen,
    ramjet_spillage_momentum_scale_n,
)
from .atmosphere import G0_M_PER_S2, standard_atmosphere
from .config import ReferenceCase
from .flight import PointMassDerivative, PointMassState, longitudinal_derivative_from_forces
from .performance_maps import RectilinearEngineMap
from .ramjet import evaluate_ramjet


class MissionPhase(str, Enum):
    PULSEJET_CLIMB = "pulsejet_climb"
    PULSEJET_DIVE = "pulsejet_dive"
    RAMJET_ACCELERATION = "ramjet_acceleration"
    RAMJET_RUN = "ramjet_run"
    ZOOM_CLIMB = "zoom_climb"
    GLIDE = "glide"
    COMPLETE = "complete"
    ABORTED = "aborted"


@dataclass(frozen=True)
class MissionPolicy:
    top_of_climb_altitude_m: float
    speed_run_altitude_m: float
    climb_flight_path_angle_rad: float
    dive_flight_path_angle_rad: float
    zoom_flight_path_angle_rad: float
    zoom_end_mach: float
    ramjet_handoff_mach: float
    allow_forced_ramjet_below_self_sustaining: bool
    pulsejet_propulsion_derate_fraction: float
    ramjet_propulsion_derate_fraction: float
    ramjet_spillage_drag_momentum_fraction: float
    flight_path_control_gain_per_s: float
    speed_control_gain_per_s: float
    minimum_angle_of_attack_rad: float
    maximum_angle_of_attack_rad: float

    @classmethod
    def from_case(
        cls,
        case: ReferenceCase,
        *,
        top_of_climb_altitude_m: float | None = None,
        climb_flight_path_angle_deg: float | None = None,
        dive_flight_path_angle_deg: float | None = None,
        ramjet_spillage_drag_momentum_fraction: float | None = None,
        ramjet_handoff_mach: float | None = None,
        allow_forced_ramjet_below_self_sustaining: bool | None = None,
    ) -> "MissionPolicy":
        config = case.mission_simulation
        selected_handoff_mach = (
            config.ramjet_handoff_mach
            if ramjet_handoff_mach is None
            else ramjet_handoff_mach
        )
        selected_forced_operation = (
            config.allow_forced_ramjet_below_self_sustaining
            if allow_forced_ramjet_below_self_sustaining is None
            else allow_forced_ramjet_below_self_sustaining
        )
        if (
            selected_handoff_mach < case.ramjet.minimum_self_sustaining_mach
            and not selected_forced_operation
        ):
            raise ValueError(
                "handoff below the configured self-sustaining Mach requires "
                "explicit forced-operation opt-in"
            )
        return cls(
            top_of_climb_altitude_m=(
                config.reference_top_of_climb_altitude_m
                if top_of_climb_altitude_m is None
                else top_of_climb_altitude_m
            ),
            speed_run_altitude_m=case.mission.speed_run_altitude_msl_m,
            climb_flight_path_angle_rad=radians(
                config.climb_flight_path_angle_deg
                if climb_flight_path_angle_deg is None
                else climb_flight_path_angle_deg
            ),
            dive_flight_path_angle_rad=radians(
                config.dive_flight_path_angle_deg
                if dive_flight_path_angle_deg is None
                else dive_flight_path_angle_deg
            ),
            zoom_flight_path_angle_rad=radians(config.zoom_flight_path_angle_deg),
            zoom_end_mach=config.zoom_end_mach,
            ramjet_handoff_mach=selected_handoff_mach,
            allow_forced_ramjet_below_self_sustaining=selected_forced_operation,
            pulsejet_propulsion_derate_fraction=(
                config.pulsejet_propulsion_derate_fraction
            ),
            ramjet_propulsion_derate_fraction=(
                config.ramjet_propulsion_derate_fraction
            ),
            ramjet_spillage_drag_momentum_fraction=(
                config.ramjet_spillage_drag_momentum_fraction
                if ramjet_spillage_drag_momentum_fraction is None
                else ramjet_spillage_drag_momentum_fraction
            ),
            flight_path_control_gain_per_s=config.flight_path_control_gain_per_s,
            speed_control_gain_per_s=config.speed_control_gain_per_s,
            minimum_angle_of_attack_rad=radians(
                config.minimum_angle_of_attack_deg
            ),
            maximum_angle_of_attack_rad=radians(
                config.maximum_angle_of_attack_deg
            ),
        )

    def __post_init__(self) -> None:
        if self.top_of_climb_altitude_m <= self.speed_run_altitude_m:
            raise ValueError("top-of-climb altitude must exceed speed-run altitude")
        if self.climb_flight_path_angle_rad <= 0.0:
            raise ValueError("climb flight-path angle must be positive")
        if self.dive_flight_path_angle_rad >= 0.0:
            raise ValueError("dive flight-path angle must be negative")
        if self.zoom_flight_path_angle_rad <= 0.0:
            raise ValueError("zoom flight-path angle must be positive")
        if not 0.0 <= self.ramjet_spillage_drag_momentum_fraction < 1.0:
            raise ValueError("spillage momentum fraction must be in [0, 1)")
        if self.minimum_angle_of_attack_rad >= self.maximum_angle_of_attack_rad:
            raise ValueError("minimum angle of attack must be below maximum")
        if self.ramjet_handoff_mach <= 0.0:
            raise ValueError("ramjet handoff Mach must be positive")
        if (
            self.ramjet_handoff_mach < 1.0
            and not self.allow_forced_ramjet_below_self_sustaining
        ):
            raise ValueError(
                "subsonic ramjet handoff requires explicit forced-operation opt-in"
            )


@dataclass(frozen=True)
class MissionEvent:
    time_s: float
    name: str
    from_phase: str
    to_phase: str
    altitude_m: float
    mach: float
    note: str


@dataclass(frozen=True)
class MissionSample:
    time_s: float
    phase: str
    downrange_m: float
    altitude_m: float
    speed_m_per_s: float
    mach: float
    flight_path_angle_deg: float
    angle_of_attack_deg: float
    mass_kg: float
    pulsejet_fuel_remaining_kg: float
    ramjet_fuel_remaining_kg: float
    thrust_n: float
    throttle_fraction: float
    fuel_mass_flow_kg_per_s: float
    lift_n: float
    aerodynamic_drag_n: float
    ramjet_spillage_drag_n: float
    total_drag_n: float
    dynamic_pressure_pa: float
    total_drag_area_m2: float
    pulsejet_map_clamped: bool
    aerodynamic_table_clamped: bool


@dataclass(frozen=True)
class MissionSummary:
    termination_reason: str
    final_phase: str
    duration_s: float
    downrange_m: float
    final_altitude_m: float
    final_speed_m_per_s: float
    final_flight_path_angle_deg: float
    maximum_altitude_m: float
    maximum_mach: float
    minimum_mach: float
    minimum_speed_m_per_s: float
    maximum_dynamic_pressure_pa: float
    time_above_mach_one_s: float
    time_at_angle_of_attack_limit_s: float
    pulsejet_fuel_used_kg: float
    ramjet_fuel_used_kg: float
    total_fuel_used_kg: float
    top_of_climb_time_s: float | None
    ramjet_lightoff_test_crossing_time_s: float | None
    ramjet_handoff_time_s: float | None
    ramjet_run_end_time_s: float | None
    ground_contact_time_s: float | None
    minimum_ramjet_full_throttle_margin_n: float | None
    minimum_ramjet_acceleration_full_throttle_margin_n: float | None
    minimum_ramjet_run_full_throttle_margin_n: float | None
    reached_target_peak_mach: bool
    exceeded_minimum_supersonic_duration: bool
    landed_in_longitudinal_model: bool
    intact_landing_not_evaluated: bool
    reciprocal_return_not_evaluated: bool
    full_mission_numerically_closes: bool
    numerical_reference_only: bool
    status: tuple[str, ...]


@dataclass(frozen=True)
class MissionResult:
    launch_screen: LaunchLiftScreen
    policy: MissionPolicy
    summary: MissionSummary
    events: tuple[MissionEvent, ...]
    samples: tuple[MissionSample, ...]


@dataclass(frozen=True)
class _ControlResult:
    thrust_n: float
    throttle_fraction: float
    fuel_mass_flow_kg_per_s: float
    angle_of_attack_rad: float
    aero: AerodynamicForces
    spillage_drag_n: float
    full_throttle_margin_n: float | None
    pulsejet_map_clamped: bool
    ramjet_self_sustaining_candidate: bool

    @property
    def total_drag_n(self) -> float:
        return self.aero.drag_n + self.spillage_drag_n


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


class MissionSimulator:
    """Integrate the configured mission with explicit phase and fuel events."""

    def __init__(
        self,
        case: ReferenceCase,
        pulsejet_map: RectilinearEngineMap,
        *,
        policy: MissionPolicy | None = None,
        aero_model: LongitudinalAeroModel | None = None,
    ) -> None:
        self.case = case
        self.pulsejet_map = pulsejet_map
        self.policy = MissionPolicy.from_case(case) if policy is None else policy
        self.aero_model = BudgetAeroModel(case) if aero_model is None else aero_model
        if self.policy.top_of_climb_altitude_m < (
            case.mission.top_of_climb_altitude_min_msl_m
        ) or self.policy.top_of_climb_altitude_m > (
            case.mission.top_of_climb_altitude_max_msl_m
        ):
            raise ValueError("policy top-of-climb altitude lies outside mission bounds")

    def _mach(self, state: PointMassState) -> float:
        atmosphere = standard_atmosphere(state.altitude_m)
        return state.speed_m_per_s / atmosphere.speed_of_sound_m_per_s

    def _target_gamma_alpha(
        self,
        state: PointMassState,
        target_gamma_rad: float,
        thrust_n: float,
    ) -> float:
        atmosphere = standard_atmosphere(state.altitude_m)
        dynamic_pressure_pa = (
            0.5 * atmosphere.density_kg_per_m3 * state.speed_m_per_s**2
        )
        lift_scale_n = dynamic_pressure_pa * self.case.flight.reference_area_m2
        if lift_scale_n <= 1e-12:
            return self.policy.maximum_angle_of_attack_rad
        desired_gamma_rate_rad_per_s = (
            self.policy.flight_path_control_gain_per_s
            * (target_gamma_rad - state.flight_path_angle_rad)
        )
        angle_of_attack_rad = 0.0
        for _ in range(3):
            required_lift_n = (
                state.mass_kg
                * state.speed_m_per_s
                * desired_gamma_rate_rad_per_s
                + state.mass_kg
                * G0_M_PER_S2
                * cos(state.flight_path_angle_rad)
                - thrust_n * sin(angle_of_attack_rad)
            )
            required_lift_coefficient = required_lift_n / lift_scale_n
            angle_of_attack_rad = _clamp(
                self.aero_model.angle_of_attack_for_lift_coefficient(
                    state.speed_m_per_s / atmosphere.speed_of_sound_m_per_s,
                    required_lift_coefficient,
                ),
                self.policy.minimum_angle_of_attack_rad,
                self.policy.maximum_angle_of_attack_rad,
            )
        return angle_of_attack_rad

    def _ramjet_control(
        self,
        state: PointMassState,
        *,
        accelerate: bool,
    ) -> _ControlResult:
        mach = self._mach(state)
        result = evaluate_ramjet(
            self.case.ramjet,
            self.case.selector,
            self.case.nozzle,
            self.case.fuel,
            state.altitude_m,
            mach,
        )
        full_thrust_n = (
            1.0 - self.policy.ramjet_propulsion_derate_fraction
        ) * result.net_thrust_n
        spillage_drag_n = (
            self.policy.ramjet_spillage_drag_momentum_fraction
            * ramjet_spillage_momentum_scale_n(result, state.altitude_m)
        )
        target_gamma_rad = self.policy.flight_path_control_gain_per_s * (
            self.policy.speed_run_altitude_m - state.altitude_m
        ) / max(state.speed_m_per_s, 1e-9)
        target_gamma_rad = _clamp(
            target_gamma_rad,
            self.policy.dive_flight_path_angle_rad,
            self.policy.climb_flight_path_angle_rad,
        )
        if accelerate:
            angle_of_attack_rad = self._target_gamma_alpha(
                state,
                target_gamma_rad,
                full_thrust_n,
            )
            aero = self.aero_model.forces(state, angle_of_attack_rad)
            return _ControlResult(
                thrust_n=full_thrust_n,
                throttle_fraction=1.0,
                fuel_mass_flow_kg_per_s=result.fuel_mass_flow_kg_per_s,
                angle_of_attack_rad=angle_of_attack_rad,
                aero=aero,
                spillage_drag_n=spillage_drag_n,
                full_throttle_margin_n=(
                    full_thrust_n - aero.drag_n - spillage_drag_n
                ),
                pulsejet_map_clamped=False,
                ramjet_self_sustaining_candidate=(
                    result.self_sustaining_candidate
                ),
            )
        throttle_fraction = 1.0
        angle_of_attack_rad = self._target_gamma_alpha(
            state,
            target_gamma_rad,
            full_thrust_n,
        )
        for _ in range(3):
            aero = self.aero_model.forces(state, angle_of_attack_rad)
            atmosphere = standard_atmosphere(state.altitude_m)
            target_speed_m_per_s = (
                self.case.mission.peak_mach * atmosphere.speed_of_sound_m_per_s
            )
            desired_acceleration_m_per_s2 = (
                self.policy.speed_control_gain_per_s
                * (target_speed_m_per_s - state.speed_m_per_s)
            )
            required_thrust_n = (
                aero.drag_n
                + spillage_drag_n
                + state.mass_kg
                * G0_M_PER_S2
                * sin(state.flight_path_angle_rad)
                + state.mass_kg * desired_acceleration_m_per_s2
            ) / max(cos(angle_of_attack_rad), 1e-6)
            throttle_fraction = (
                _clamp(required_thrust_n / full_thrust_n, 0.0, 1.0)
                if full_thrust_n > 0.0
                else 0.0
            )
            thrust_n = full_thrust_n * throttle_fraction
            angle_of_attack_rad = self._target_gamma_alpha(
                state,
                target_gamma_rad,
                thrust_n,
            )
        aero = self.aero_model.forces(state, angle_of_attack_rad)
        thrust_n = full_thrust_n * throttle_fraction
        return _ControlResult(
            thrust_n=thrust_n,
            throttle_fraction=throttle_fraction,
            fuel_mass_flow_kg_per_s=(
                result.fuel_mass_flow_kg_per_s * throttle_fraction
            ),
            angle_of_attack_rad=angle_of_attack_rad,
            aero=aero,
            spillage_drag_n=spillage_drag_n,
            full_throttle_margin_n=(
                full_thrust_n - aero.drag_n - spillage_drag_n
            ),
            pulsejet_map_clamped=False,
            ramjet_self_sustaining_candidate=result.self_sustaining_candidate,
        )

    def _control(self, state: PointMassState, phase: MissionPhase) -> _ControlResult:
        mach = self._mach(state)
        if phase in {MissionPhase.PULSEJET_CLIMB, MissionPhase.PULSEJET_DIVE}:
            map_value = self.pulsejet_map.evaluate(state.altitude_m, mach)
            thrust_n = (
                1.0 - self.policy.pulsejet_propulsion_derate_fraction
            ) * map_value.net_thrust_n
            target_gamma_rad = (
                self.policy.climb_flight_path_angle_rad
                if phase is MissionPhase.PULSEJET_CLIMB
                else self.policy.dive_flight_path_angle_rad
            )
            angle_of_attack_rad = self._target_gamma_alpha(
                state,
                target_gamma_rad,
                thrust_n,
            )
            return _ControlResult(
                thrust_n=thrust_n,
                throttle_fraction=1.0,
                fuel_mass_flow_kg_per_s=map_value.fuel_mass_flow_kg_per_s,
                angle_of_attack_rad=angle_of_attack_rad,
                aero=self.aero_model.forces(state, angle_of_attack_rad),
                spillage_drag_n=0.0,
                full_throttle_margin_n=None,
                pulsejet_map_clamped=map_value.clamped_to_map_boundary,
                ramjet_self_sustaining_candidate=False,
            )
        if phase in {MissionPhase.RAMJET_ACCELERATION, MissionPhase.RAMJET_RUN}:
            return self._ramjet_control(
                state,
                accelerate=phase is MissionPhase.RAMJET_ACCELERATION,
            )

        if phase is MissionPhase.ZOOM_CLIMB:
            angle_of_attack_rad = self._target_gamma_alpha(
                state,
                self.policy.zoom_flight_path_angle_rad,
                0.0,
            )
        else:
            best_glide_angle_of_attack_rad = _clamp(
                self.aero_model.best_glide_angle_of_attack_rad(mach),
                self.policy.minimum_angle_of_attack_rad,
                self.policy.maximum_angle_of_attack_rad,
            )
            best_glide_forces = self.aero_model.forces(
                state,
                best_glide_angle_of_attack_rad,
            )
            lift_to_drag_ratio = (
                best_glide_forces.lift_n / best_glide_forces.drag_n
                if best_glide_forces.drag_n > 0.0
                else 0.0
            )
            target_glide_gamma_rad = (
                -atan(1.0 / lift_to_drag_ratio)
                if lift_to_drag_ratio > 0.0
                else self.policy.dive_flight_path_angle_rad
            )
            angle_of_attack_rad = self._target_gamma_alpha(
                state,
                target_glide_gamma_rad,
                0.0,
            )
        return _ControlResult(
            thrust_n=0.0,
            throttle_fraction=0.0,
            fuel_mass_flow_kg_per_s=0.0,
            angle_of_attack_rad=angle_of_attack_rad,
            aero=self.aero_model.forces(state, angle_of_attack_rad),
            spillage_drag_n=0.0,
            full_throttle_margin_n=None,
            pulsejet_map_clamped=False,
            ramjet_self_sustaining_candidate=False,
        )

    @staticmethod
    def _derivative(
        state: PointMassState,
        control: _ControlResult,
    ) -> PointMassDerivative:
        return longitudinal_derivative_from_forces(
            state,
            thrust_n=control.thrust_n,
            fuel_mass_flow_kg_per_s=control.fuel_mass_flow_kg_per_s,
            angle_of_attack_rad=control.angle_of_attack_rad,
            lift_n=control.aero.lift_n,
            drag_n=control.total_drag_n,
        )

    @staticmethod
    def _advance(
        state: PointMassState,
        derivative: PointMassDerivative,
        time_step_s: float,
    ) -> PointMassState:
        return PointMassState(
            downrange_m=(
                state.downrange_m
                + time_step_s * derivative.downrange_rate_m_per_s
            ),
            altitude_m=state.altitude_m + time_step_s * derivative.climb_rate_m_per_s,
            speed_m_per_s=max(
                state.speed_m_per_s
                + time_step_s * derivative.acceleration_m_per_s2,
                1e-6,
            ),
            flight_path_angle_rad=(
                state.flight_path_angle_rad
                + time_step_s * derivative.flight_path_rate_rad_per_s
            ),
            mass_kg=state.mass_kg + time_step_s * derivative.mass_rate_kg_per_s,
        )

    @staticmethod
    def _fuel_limited_control(
        control: _ControlResult,
        fuel_remaining_kg: float,
        time_step_s: float,
    ) -> _ControlResult:
        requested_fuel_kg = control.fuel_mass_flow_kg_per_s * time_step_s
        if requested_fuel_kg <= fuel_remaining_kg + 1e-15:
            return control
        if requested_fuel_kg <= 0.0:
            return control
        fraction = max(fuel_remaining_kg, 0.0) / requested_fuel_kg
        return replace(
            control,
            thrust_n=control.thrust_n * fraction,
            throttle_fraction=control.throttle_fraction * fraction,
            fuel_mass_flow_kg_per_s=max(fuel_remaining_kg, 0.0) / time_step_s,
            spillage_drag_n=control.spillage_drag_n * fraction,
        )

    def run(
        self,
        *,
        release_speed_m_per_s: float | None = None,
        integration_time_step_s: float | None = None,
        maximum_duration_s: float | None = None,
    ) -> MissionResult:
        time_step_s = (
            self.case.mission_simulation.integration_time_step_s
            if integration_time_step_s is None
            else integration_time_step_s
        )
        maximum_duration = (
            self.case.mission_simulation.maximum_duration_s
            if maximum_duration_s is None
            else maximum_duration_s
        )
        if time_step_s <= 0.0 or maximum_duration <= 0.0:
            raise ValueError("mission time step and maximum duration must be positive")
        release_speed = (
            self.case.mission_simulation.reference_release_speed_m_per_s
            if release_speed_m_per_s is None
            else release_speed_m_per_s
        )
        launch_screen = launch_lift_screen(self.case, release_speed)
        state = PointMassState(
            downrange_m=0.0,
            altitude_m=self.case.mission.field_elevation_msl_m,
            speed_m_per_s=release_speed,
            flight_path_angle_rad=0.0,
            mass_kg=self.case.flight.initial_mass_kg,
        )
        phase = MissionPhase.PULSEJET_CLIMB
        pulsejet_initial_fuel_kg = (
            self.case.mission.loaded_fuel_mass_kg
            - self.case.mission.ramjet_speed_run_fuel_budget_kg
        )
        pulsejet_fuel_remaining_kg = pulsejet_initial_fuel_kg
        ramjet_initial_fuel_kg = self.case.mission.ramjet_speed_run_fuel_budget_kg
        ramjet_fuel_remaining_kg = ramjet_initial_fuel_kg
        time_s = 0.0
        time_above_mach_one_s = 0.0
        maximum_mach = self._mach(state)
        minimum_mach = maximum_mach
        minimum_speed_m_per_s = state.speed_m_per_s
        maximum_altitude_m = state.altitude_m
        maximum_dynamic_pressure_pa = 0.0
        time_at_angle_of_attack_limit_s = 0.0
        minimum_ramjet_margin_n: float | None = None
        minimum_ramjet_acceleration_margin_n: float | None = None
        minimum_ramjet_run_margin_n: float | None = None
        top_of_climb_time_s: float | None = None
        lightoff_time_s: float | None = None
        handoff_time_s: float | None = None
        ramjet_run_end_time_s: float | None = None
        ground_contact_time_s: float | None = None
        termination_reason = "maximum_duration_reached"
        failures: list[str] = []
        samples: list[MissionSample] = []
        events: list[MissionEvent] = [
            MissionEvent(
                time_s=0.0,
                name="sled_release",
                from_phase="sled",
                to_phase=phase.value,
                altitude_m=state.altitude_m,
                mach=self._mach(state),
                note=(
                    "configured mission-simulation reference release speed"
                    if release_speed_m_per_s is None
                    else "release speed was supplied as a mission-trade input"
                ),
            )
        ]

        def transition(
            new_phase: MissionPhase,
            name: str,
            note: str,
        ) -> None:
            nonlocal phase
            events.append(
                MissionEvent(
                    time_s=time_s,
                    name=name,
                    from_phase=phase.value,
                    to_phase=new_phase.value,
                    altitude_m=state.altitude_m,
                    mach=self._mach(state),
                    note=note,
                )
            )
            phase = new_phase

        while time_s < maximum_duration - 0.5 * time_step_s:
            if phase in {MissionPhase.COMPLETE, MissionPhase.ABORTED}:
                break
            active_fuel_remaining_kg = (
                pulsejet_fuel_remaining_kg
                if phase in {MissionPhase.PULSEJET_CLIMB, MissionPhase.PULSEJET_DIVE}
                else ramjet_fuel_remaining_kg
                if phase in {MissionPhase.RAMJET_ACCELERATION, MissionPhase.RAMJET_RUN}
                else 0.0
            )
            control_start = self._fuel_limited_control(
                self._control(state, phase),
                active_fuel_remaining_kg,
                time_step_s,
            )
            derivative_start = self._derivative(state, control_start)
            midpoint_state = self._advance(state, derivative_start, 0.5 * time_step_s)
            control_midpoint = self._fuel_limited_control(
                self._control(midpoint_state, phase),
                active_fuel_remaining_kg,
                time_step_s,
            )
            derivative_midpoint = self._derivative(midpoint_state, control_midpoint)
            previous_state = state
            state = self._advance(state, derivative_midpoint, time_step_s)
            fuel_used_kg = control_midpoint.fuel_mass_flow_kg_per_s * time_step_s
            if phase in {MissionPhase.PULSEJET_CLIMB, MissionPhase.PULSEJET_DIVE}:
                pulsejet_fuel_remaining_kg = max(
                    pulsejet_fuel_remaining_kg - fuel_used_kg,
                    0.0,
                )
            elif phase in {MissionPhase.RAMJET_ACCELERATION, MissionPhase.RAMJET_RUN}:
                ramjet_fuel_remaining_kg = max(
                    ramjet_fuel_remaining_kg - fuel_used_kg,
                    0.0,
                )
            time_s += time_step_s
            previous_mach = self._mach(previous_state)
            mach = self._mach(state)
            midpoint_mach = self._mach(midpoint_state)
            if midpoint_mach > 1.0:
                time_above_mach_one_s += time_step_s
            maximum_mach = max(maximum_mach, mach, midpoint_mach)
            minimum_mach = min(minimum_mach, mach, midpoint_mach)
            minimum_speed_m_per_s = min(
                minimum_speed_m_per_s,
                state.speed_m_per_s,
                midpoint_state.speed_m_per_s,
            )
            maximum_altitude_m = max(
                maximum_altitude_m,
                state.altitude_m,
                midpoint_state.altitude_m,
            )
            maximum_dynamic_pressure_pa = max(
                maximum_dynamic_pressure_pa,
                control_midpoint.aero.dynamic_pressure_pa,
            )
            if (
                abs(
                    control_midpoint.angle_of_attack_rad
                    - self.policy.minimum_angle_of_attack_rad
                )
                <= 1.0e-10
                or abs(
                    control_midpoint.angle_of_attack_rad
                    - self.policy.maximum_angle_of_attack_rad
                )
                <= 1.0e-10
            ):
                time_at_angle_of_attack_limit_s += time_step_s
            if control_midpoint.full_throttle_margin_n is not None:
                minimum_ramjet_margin_n = (
                    control_midpoint.full_throttle_margin_n
                    if minimum_ramjet_margin_n is None
                    else min(
                        minimum_ramjet_margin_n,
                        control_midpoint.full_throttle_margin_n,
                    )
                )
                if phase is MissionPhase.RAMJET_ACCELERATION:
                    minimum_ramjet_acceleration_margin_n = (
                        control_midpoint.full_throttle_margin_n
                        if minimum_ramjet_acceleration_margin_n is None
                        else min(
                            minimum_ramjet_acceleration_margin_n,
                            control_midpoint.full_throttle_margin_n,
                        )
                    )
                elif phase is MissionPhase.RAMJET_RUN:
                    minimum_ramjet_run_margin_n = (
                        control_midpoint.full_throttle_margin_n
                        if minimum_ramjet_run_margin_n is None
                        else min(
                            minimum_ramjet_run_margin_n,
                            control_midpoint.full_throttle_margin_n,
                        )
                    )
            samples.append(
                MissionSample(
                    time_s=time_s,
                    phase=phase.value,
                    downrange_m=state.downrange_m,
                    altitude_m=state.altitude_m,
                    speed_m_per_s=state.speed_m_per_s,
                    mach=mach,
                    flight_path_angle_deg=degrees(state.flight_path_angle_rad),
                    angle_of_attack_deg=degrees(
                        control_midpoint.angle_of_attack_rad
                    ),
                    mass_kg=state.mass_kg,
                    pulsejet_fuel_remaining_kg=pulsejet_fuel_remaining_kg,
                    ramjet_fuel_remaining_kg=ramjet_fuel_remaining_kg,
                    thrust_n=control_midpoint.thrust_n,
                    throttle_fraction=control_midpoint.throttle_fraction,
                    fuel_mass_flow_kg_per_s=(
                        control_midpoint.fuel_mass_flow_kg_per_s
                    ),
                    lift_n=control_midpoint.aero.lift_n,
                    aerodynamic_drag_n=control_midpoint.aero.drag_n,
                    ramjet_spillage_drag_n=control_midpoint.spillage_drag_n,
                    total_drag_n=control_midpoint.total_drag_n,
                    dynamic_pressure_pa=control_midpoint.aero.dynamic_pressure_pa,
                    total_drag_area_m2=control_midpoint.aero.total_drag_area_m2,
                    pulsejet_map_clamped=(
                        control_midpoint.pulsejet_map_clamped
                    ),
                    aerodynamic_table_clamped=(
                        control_midpoint.aero.solver_table_clamped
                    ),
                )
            )

            if (
                lightoff_time_s is None
                and previous_mach < self.case.ramjet.minimum_lightoff_test_mach <= mach
            ):
                lightoff_time_s = time_s
                events.append(
                    MissionEvent(
                        time_s=time_s,
                        name="ramjet_lightoff_test_crossing",
                        from_phase=phase.value,
                        to_phase=phase.value,
                        altitude_m=state.altitude_m,
                        mach=mach,
                        note=(
                            "forced-ignition handoff is permitted at this crossing"
                            if (
                                self.policy.allow_forced_ramjet_below_self_sustaining
                                and self.policy.ramjet_handoff_mach
                                <= self.case.ramjet.minimum_lightoff_test_mach
                            )
                            else "diagnostic crossing only; pulsejet remains selected "
                            "until the self-sustaining handoff gate"
                        ),
                    )
                )

            if state.altitude_m <= self.case.mission.field_elevation_msl_m:
                state = replace(
                    state,
                    altitude_m=self.case.mission.field_elevation_msl_m,
                )
                ground_contact_time_s = time_s
                termination_reason = "ground_contact"
                transition(
                    MissionPhase.COMPLETE,
                    "ground_contact",
                    "touchdown loads and intact landing are not evaluated",
                )
                break

            if phase is MissionPhase.PULSEJET_CLIMB and (
                state.altitude_m >= self.policy.top_of_climb_altitude_m
            ):
                top_of_climb_time_s = time_s
                transition(
                    MissionPhase.PULSEJET_DIVE,
                    "top_of_climb",
                    "begin configured powered dive",
                )
            elif phase is MissionPhase.PULSEJET_DIVE and mach >= (
                self.policy.ramjet_handoff_mach
            ):
                ramjet_at_handoff = evaluate_ramjet(
                    self.case.ramjet,
                    self.case.selector,
                    self.case.nozzle,
                    self.case.fuel,
                    state.altitude_m,
                    mach,
                )
                if (
                    ramjet_at_handoff.self_sustaining_candidate
                    or self.policy.allow_forced_ramjet_below_self_sustaining
                ):
                    handoff_time_s = time_s
                    transition(
                        (
                            MissionPhase.RAMJET_RUN
                            if mach >= self.case.mission.peak_mach
                            else MissionPhase.RAMJET_ACCELERATION
                        ),
                        "ramjet_handoff",
                        (
                            "selector changes paths with forced ignition below the "
                            "self-sustaining gate"
                            if not ramjet_at_handoff.self_sustaining_candidate
                            else "selector changes paths at the self-sustaining candidate gate"
                        ),
                    )
                else:
                    failures.append("ramjet_not_self_sustaining_at_handoff")
                    transition(
                        MissionPhase.GLIDE,
                        "failed_ramjet_handoff",
                        "low-order operability gate rejected the handoff point",
                    )
            elif phase in {MissionPhase.RAMJET_ACCELERATION, MissionPhase.RAMJET_RUN}:
                if (
                    not control_midpoint.ramjet_self_sustaining_candidate
                    and not self.policy.allow_forced_ramjet_below_self_sustaining
                ):
                    failures.append("ramjet_fell_below_self_sustaining_gate")
                    ramjet_run_end_time_s = time_s
                    transition(
                        MissionPhase.GLIDE,
                        "ramjet_operability_exit",
                        "ramjet state fell below the configured candidate gate",
                    )
                elif (
                    phase is MissionPhase.RAMJET_ACCELERATION
                    and mach >= self.case.mission.peak_mach
                ):
                    transition(
                        MissionPhase.RAMJET_RUN,
                        "target_peak_mach_reached",
                        "begin fuel-limited Mach target hold",
                    )
                elif ramjet_fuel_remaining_kg <= 1e-12:
                    if phase is MissionPhase.RAMJET_ACCELERATION:
                        failures.append("ramjet_fuel_depleted_before_target_mach")
                    ramjet_run_end_time_s = time_s
                    transition(
                        MissionPhase.ZOOM_CLIMB,
                        "ramjet_fuel_depleted",
                        "speed run ends from fuel depletion, not a prescribed time",
                    )
            elif phase is MissionPhase.ZOOM_CLIMB and mach <= self.policy.zoom_end_mach:
                transition(
                    MissionPhase.GLIDE,
                    "zoom_end",
                    "transition to best-glide angle of attack",
                )

            if (
                phase in {MissionPhase.PULSEJET_CLIMB, MissionPhase.PULSEJET_DIVE}
                and pulsejet_fuel_remaining_kg <= 1e-12
            ):
                failures.append("pulsejet_fuel_depleted_before_ramjet_handoff")
                transition(
                    MissionPhase.GLIDE,
                    "pulsejet_fuel_depleted",
                    "reserved ramjet fuel remains isolated from the pulsejet budget",
                )

        if phase not in {MissionPhase.COMPLETE, MissionPhase.ABORTED}:
            phase = MissionPhase.ABORTED
            failures.append("maximum_mission_duration_reached")
        pulsejet_fuel_used_kg = (
            pulsejet_initial_fuel_kg - pulsejet_fuel_remaining_kg
        )
        ramjet_fuel_used_kg = ramjet_initial_fuel_kg - ramjet_fuel_remaining_kg
        reached_target_peak_mach = maximum_mach >= self.case.mission.peak_mach
        duration_pass = (
            time_above_mach_one_s
            >= self.case.requirements.minimum_time_above_mach_one_s
        )
        landed = ground_contact_time_s is not None
        status = list(dict.fromkeys(failures))
        if not launch_screen.lift_closes_at_release:
            status.append("configured_lifting_area_does_not_close_release_lift")
        if any(sample.pulsejet_map_clamped for sample in samples):
            status.append("pulsejet_map_boundary_clamping_occurred")
        if any(sample.aerodynamic_table_clamped for sample in samples):
            status.append("aerodynamic_table_boundary_clamping_occurred")
        if time_at_angle_of_attack_limit_s > 0.0:
            status.append("angle_of_attack_limit_saturation_occurred")
        if self.policy.ramjet_spillage_drag_momentum_fraction == 0.0:
            status.append("ramjet_spillage_drag_is_zero_penalty_optimistic_bound")
        if self.policy.allow_forced_ramjet_below_self_sustaining:
            status.append("forced_ramjet_operation_below_self_sustaining_gate_assumed")
        if not reached_target_peak_mach:
            status.append("target_peak_mach_not_reached")
        if not duration_pass:
            status.append("minimum_supersonic_duration_not_reached")
        status.extend(
            (
                "intact_landing_loads_not_evaluated",
                "reciprocal_turn_and_return_not_evaluated",
                "stall_and_control_moment_limits_not_evaluated",
                "pulsejet_map_is_numerical_reference_only",
            )
        )
        status.extend(self.aero_model.status_tags)
        full_mission_closes = (
            landed
            and reached_target_peak_mach
            and duration_pass
            and not failures
            and launch_screen.lift_closes_at_release
        )
        return MissionResult(
            launch_screen=launch_screen,
            policy=self.policy,
            summary=MissionSummary(
                termination_reason=termination_reason,
                final_phase=phase.value,
                duration_s=time_s,
                downrange_m=state.downrange_m,
                final_altitude_m=state.altitude_m,
                final_speed_m_per_s=state.speed_m_per_s,
                final_flight_path_angle_deg=degrees(
                    state.flight_path_angle_rad
                ),
                maximum_altitude_m=maximum_altitude_m,
                maximum_mach=maximum_mach,
                minimum_mach=minimum_mach,
                minimum_speed_m_per_s=minimum_speed_m_per_s,
                maximum_dynamic_pressure_pa=maximum_dynamic_pressure_pa,
                time_above_mach_one_s=time_above_mach_one_s,
                time_at_angle_of_attack_limit_s=(
                    time_at_angle_of_attack_limit_s
                ),
                pulsejet_fuel_used_kg=pulsejet_fuel_used_kg,
                ramjet_fuel_used_kg=ramjet_fuel_used_kg,
                total_fuel_used_kg=pulsejet_fuel_used_kg + ramjet_fuel_used_kg,
                top_of_climb_time_s=top_of_climb_time_s,
                ramjet_lightoff_test_crossing_time_s=lightoff_time_s,
                ramjet_handoff_time_s=handoff_time_s,
                ramjet_run_end_time_s=ramjet_run_end_time_s,
                ground_contact_time_s=ground_contact_time_s,
                minimum_ramjet_full_throttle_margin_n=minimum_ramjet_margin_n,
                minimum_ramjet_acceleration_full_throttle_margin_n=(
                    minimum_ramjet_acceleration_margin_n
                ),
                minimum_ramjet_run_full_throttle_margin_n=(
                    minimum_ramjet_run_margin_n
                ),
                reached_target_peak_mach=reached_target_peak_mach,
                exceeded_minimum_supersonic_duration=duration_pass,
                landed_in_longitudinal_model=landed,
                intact_landing_not_evaluated=True,
                reciprocal_return_not_evaluated=True,
                full_mission_numerically_closes=full_mission_closes,
                numerical_reference_only=True,
                status=tuple(dict.fromkeys(status)),
            ),
            events=tuple(events),
            samples=tuple(samples),
        )
