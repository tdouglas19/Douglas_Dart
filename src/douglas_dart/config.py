"""Validated configuration objects and YAML loading."""

from __future__ import annotations

from dataclasses import dataclass
from math import isclose, pi
from pathlib import Path
from typing import Any, Mapping

import yaml


def _positive(name: str, value: float) -> float:
    value = float(value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return value


def _fraction(name: str, value: float, *, inclusive_one: bool = True) -> float:
    value = float(value)
    upper_ok = value <= 1.0 if inclusive_one else value < 1.0
    if value <= 0.0 or not upper_ok:
        interval = "(0, 1]" if inclusive_one else "(0, 1)"
        raise ValueError(f"{name} must be in {interval}")
    return value


@dataclass(frozen=True)
class Fuel:
    key: str
    display_name: str
    lower_heating_value_j_per_kg: float
    stoichiometric_air_fuel_ratio: float
    density_kg_per_m3: float
    source_status: str

    def __post_init__(self) -> None:
        _positive("lower_heating_value_j_per_kg", self.lower_heating_value_j_per_kg)
        _positive("stoichiometric_air_fuel_ratio", self.stoichiometric_air_fuel_ratio)
        _positive("density_kg_per_m3", self.density_kg_per_m3)


@dataclass(frozen=True)
class SelectorConfig:
    """``ramjet_total_pressure_recovery`` is an *installed-efficiency* factor.

    ``ramjet.py`` multiplies it by the idealized, Mach-dependent normal-shock
    recovery (``ideal_inlet_shock_recovery``, 1.0 below Mach 1) rather than using it
    as the total recovery outright. It represents everything the idealized shock
    model does not capture -- duct friction, bends, boundary-layer bleed, and this
    vehicle's own pulsejet/ramjet selector losses -- and stays roughly constant
    with Mach, unlike the idealized shock term.
    """

    circular_intake_diameter_m: float
    open_fraction: float
    discharge_coefficient: float
    pulsejet_total_pressure_recovery: float
    ramjet_total_pressure_recovery: float

    def __post_init__(self) -> None:
        _positive("circular_intake_diameter_m", self.circular_intake_diameter_m)
        if not 0.0 < self.open_fraction <= 0.5:
            raise ValueError(
                "open_fraction must be in (0, 0.5] for mutually exclusive half-intakes"
        )
        _fraction("discharge_coefficient", self.discharge_coefficient)
        _fraction(
            "pulsejet_total_pressure_recovery",
            self.pulsejet_total_pressure_recovery,
        )
        _fraction(
            "ramjet_total_pressure_recovery",
            self.ramjet_total_pressure_recovery,
        )

    @property
    def circular_area_m2(self) -> float:
        return pi * self.circular_intake_diameter_m**2 / 4.0

    @property
    def available_area_m2(self) -> float:
        return self.open_fraction * self.circular_area_m2


@dataclass(frozen=True)
class NozzleConfig:
    throat_diameter_m: float
    exit_to_throat_area_ratio: float
    discharge_coefficient: float

    def __post_init__(self) -> None:
        _positive("throat_diameter_m", self.throat_diameter_m)
        if self.exit_to_throat_area_ratio < 1.0:
            raise ValueError("exit_to_throat_area_ratio must be at least one")
        _fraction("discharge_coefficient", self.discharge_coefficient)

    @property
    def throat_area_m2(self) -> float:
        return pi * self.throat_diameter_m**2 / 4.0

    @property
    def exit_area_m2(self) -> float:
        return self.throat_area_m2 * self.exit_to_throat_area_ratio


@dataclass(frozen=True)
class PulsejetConfig:
    chamber_volume_m3: float
    gamma: float
    gas_constant_j_per_kg_k: float
    combustion_efficiency: float
    target_equivalence_ratio: float
    initial_equivalence_ratio: float
    burn_duration_s: float
    minimum_cycle_period_s: float
    ignition_pressure_ratio_max: float
    minimum_fresh_air_fraction: float
    wall_temperature_k: float
    wall_heat_transfer_w_per_k: float
    maximum_gas_temperature_k: float

    def __post_init__(self) -> None:
        for name in (
            "chamber_volume_m3",
            "gas_constant_j_per_kg_k",
            "target_equivalence_ratio",
            "initial_equivalence_ratio",
            "burn_duration_s",
            "minimum_cycle_period_s",
            "ignition_pressure_ratio_max",
            "minimum_fresh_air_fraction",
            "wall_temperature_k",
            "maximum_gas_temperature_k",
        ):
            _positive(name, getattr(self, name))
        if self.gamma <= 1.0:
            raise ValueError("gamma must exceed one")
        _fraction("combustion_efficiency", self.combustion_efficiency)
        if self.wall_heat_transfer_w_per_k < 0.0:
            raise ValueError("wall_heat_transfer_w_per_k cannot be negative")
        if self.maximum_gas_temperature_k <= self.wall_temperature_k:
            raise ValueError("maximum gas temperature must exceed wall temperature")


@dataclass(frozen=True)
class RamjetConfig:
    gamma: float
    gas_constant_j_per_kg_k: float
    mass_capture_coefficient: float
    combustor_total_pressure_loss_fraction: float
    combustor_efficiency: float
    target_combustor_exit_temperature_k: float
    minimum_lightoff_test_mach: float
    minimum_self_sustaining_mach: float

    def __post_init__(self) -> None:
        if self.gamma <= 1.0:
            raise ValueError("gamma must exceed one")
        _positive("gas_constant_j_per_kg_k", self.gas_constant_j_per_kg_k)
        _fraction("mass_capture_coefficient", self.mass_capture_coefficient)
        _fraction("combustor_efficiency", self.combustor_efficiency)
        if not 0.0 <= self.combustor_total_pressure_loss_fraction < 1.0:
            raise ValueError("combustor pressure loss must be in [0, 1)")
        _positive(
            "target_combustor_exit_temperature_k", self.target_combustor_exit_temperature_k
        )
        _positive("minimum_lightoff_test_mach", self.minimum_lightoff_test_mach)
        _positive("minimum_self_sustaining_mach", self.minimum_self_sustaining_mach)
        if self.minimum_self_sustaining_mach < self.minimum_lightoff_test_mach:
            raise ValueError("self-sustaining Mach cannot be below the light-off test Mach")


@dataclass(frozen=True)
class FlightConfig:
    reference_area_m2: float
    initial_mass_kg: float
    zero_lift_drag_coefficient: float
    induced_drag_factor: float
    lift_curve_slope_per_rad: float
    maximum_lift_coefficient: float

    def __post_init__(self) -> None:
        for name in (
            "reference_area_m2",
            "initial_mass_kg",
            "zero_lift_drag_coefficient",
            "induced_drag_factor",
            "lift_curve_slope_per_rad",
            "maximum_lift_coefficient",
        ):
            _positive(name, getattr(self, name))


@dataclass(frozen=True)
class VehicleConfig:
    """Outer mold-line inputs kept separate from the selector intake geometry."""

    body_diameter_m: float
    body_length_m: float
    drag_area_reference_body_diameter_m: float
    peak_mach_drag_area_ceiling_m2: float

    def __post_init__(self) -> None:
        for name in (
            "body_diameter_m",
            "body_length_m",
            "drag_area_reference_body_diameter_m",
            "peak_mach_drag_area_ceiling_m2",
        ):
            _positive(name, getattr(self, name))

    @property
    def fineness_ratio(self) -> float:
        return self.body_length_m / self.body_diameter_m


@dataclass(frozen=True)
class MissionConfig:
    """User mission requirements and recovered prior sizing allocations."""

    field_elevation_msl_m: float
    sled_release_speed_min_m_per_s: float
    sled_release_speed_max_m_per_s: float
    top_of_climb_altitude_min_msl_m: float
    top_of_climb_altitude_max_msl_m: float
    speed_run_altitude_msl_m: float
    peak_mach: float
    loaded_fuel_mass_kg: float
    ramjet_speed_run_fuel_budget_kg: float

    def __post_init__(self) -> None:
        for name in (
            "sled_release_speed_min_m_per_s",
            "sled_release_speed_max_m_per_s",
            "peak_mach",
            "loaded_fuel_mass_kg",
            "ramjet_speed_run_fuel_budget_kg",
        ):
            _positive(name, getattr(self, name))
        for name in (
            "field_elevation_msl_m",
            "top_of_climb_altitude_min_msl_m",
            "top_of_climb_altitude_max_msl_m",
            "speed_run_altitude_msl_m",
        ):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} cannot be negative")
        if self.sled_release_speed_max_m_per_s < self.sled_release_speed_min_m_per_s:
            raise ValueError("maximum sled release speed cannot be below the minimum")
        if self.top_of_climb_altitude_max_msl_m < self.top_of_climb_altitude_min_msl_m:
            raise ValueError("maximum top-of-climb altitude cannot be below the minimum")
        if self.ramjet_speed_run_fuel_budget_kg > self.loaded_fuel_mass_kg:
            raise ValueError("ramjet speed-run fuel budget cannot exceed loaded fuel mass")


@dataclass(frozen=True)
class RequirementsConfig:
    """Competition constraints kept separate from the working design point.

    Sourced from https://boomsupersonic.com/prize (fetched 2026-08-05). See
    ``docs/assumptions.md`` for the full requirement-by-requirement citation table.
    """

    maximum_takeoff_mass_kg: float
    minimum_time_above_mach_one_s: float
    minimum_peak_mach: float
    landing_intact_required: bool
    reciprocal_flight_same_day_required: bool
    pulsejet_rule_status: str
    transonic_no_altitude_loss_required: bool
    transonic_regime_start_mach: float
    remote_pilot_abort_authority_required: bool

    def __post_init__(self) -> None:
        for name in (
            "maximum_takeoff_mass_kg",
            "minimum_time_above_mach_one_s",
            "minimum_peak_mach",
            "transonic_regime_start_mach",
        ):
            _positive(name, getattr(self, name))
        if not self.pulsejet_rule_status.strip():
            raise ValueError("pulsejet rule status cannot be empty")
        if self.transonic_regime_start_mach >= self.minimum_peak_mach:
            raise ValueError(
                "transonic regime start Mach must be below the minimum peak Mach"
            )


@dataclass(frozen=True)
class SurfacePlanformConfig:
    """One repeated radial surface family for OpenVSP geometry generation."""

    x_location_m: float
    exposed_semispan_m: float
    root_chord_m: float
    tip_chord_m: float
    sweep_deg: float
    thickness_to_chord: float

    def __post_init__(self) -> None:
        if self.x_location_m < 0.0:
            raise ValueError("surface x location cannot be negative")
        for name in (
            "exposed_semispan_m",
            "root_chord_m",
            "tip_chord_m",
            "thickness_to_chord",
        ):
            _positive(name, getattr(self, name))
        if self.tip_chord_m > self.root_chord_m:
            raise ValueError("surface tip chord cannot exceed root chord")
        if not 0.0 <= self.sweep_deg < 90.0:
            raise ValueError("surface sweep must be in [0, 90) degrees")
        if self.thickness_to_chord >= 0.25:
            raise ValueError("surface thickness-to-chord must be below 0.25")

    @property
    def exposed_area_per_surface_m2(self) -> float:
        return 0.5 * (
            self.root_chord_m + self.tip_chord_m
        ) * self.exposed_semispan_m


@dataclass(frozen=True)
class ExternalShellConfig:
    """Annular fin-can shroud outside the flow-through engine body.

    Houses the fuel tank, avionics, and other auxiliary systems in the annulus
    between the inner engine flowpath and this outer mold line, per the "fin can"
    architecture: fins and lifting surfaces mount to this shell, not the inner body.
    """

    radial_offset_m: float
    start_x_m: float
    end_x_m: float
    forward_taper_length_m: float
    aft_taper_length_m: float
    tessellation: int

    def __post_init__(self) -> None:
        for name in (
            "radial_offset_m",
            "start_x_m",
            "end_x_m",
            "forward_taper_length_m",
            "aft_taper_length_m",
        ):
            _positive(name, getattr(self, name))
        if not 0.0127 - 1e-6 <= self.radial_offset_m <= 0.0254 + 1e-6:
            raise ValueError(
                "external shell radial offset should be a standard 0.5-1 inch "
                "annulus (0.0127-0.0254 m)"
            )
        if self.end_x_m <= self.start_x_m:
            raise ValueError("external shell end station must be aft of the start station")
        if (
            self.forward_taper_length_m + self.aft_taper_length_m
            >= self.end_x_m - self.start_x_m
        ):
            raise ValueError("external shell tapers cannot exceed the shell length")
        if self.tessellation < 9:
            raise ValueError("external shell tessellation is too small")


@dataclass(frozen=True)
class RamInletConfig:
    """Centered, flow-through ram-air inlet lip duct ahead of the main body's nose.

    Represents the physical inlet-lip housing for the mutually exclusive
    pulsejet/ramjet selector as a short axisymmetric flow-through duct, fairing from
    a slightly larger external lip diameter down to the main body's nose-opening
    diameter, centered on the vehicle axis -- not an off-axis decorative scoop, and
    flagged with its own OpenVSP inlet/outlet engine parameters like the main body.

    Sized from real subsonic-cowl design ratios rather than arbitrary absolute
    dimensions, so the duct scales with whatever intake diameter the selector
    trade lands on. ``lip_overshoot_fraction`` is the lip highlight's diametral
    increase over the intake diameter it fairs into -- low-drag subsonic pitot
    lips typically run 3-10% -- and ``lip_fineness_ratio`` is the duct's axial
    length per unit of that diametral overshoot the fairing has to blend out
    (higher means a more gradual fairing and lower external wave drag, at the
    cost of duct length and wetted area).
    """

    lip_overshoot_fraction: float
    lip_fineness_ratio: float
    tessellation: int

    def __post_init__(self) -> None:
        _fraction("lip_overshoot_fraction", self.lip_overshoot_fraction, inclusive_one=False)
        _positive("lip_fineness_ratio", self.lip_fineness_ratio)
        if self.tessellation < 9:
            raise ValueError("ram inlet tessellation is too small")

    def lip_diameter_m(self, intake_diameter_m: float) -> float:
        return intake_diameter_m * (1.0 + self.lip_overshoot_fraction)

    def length_m(self, intake_diameter_m: float) -> float:
        diametral_overshoot_m = intake_diameter_m * self.lip_overshoot_fraction
        return self.lip_fineness_ratio * diametral_overshoot_m


@dataclass(frozen=True)
class OpenVSPGeometryConfig:
    """External-geometry and VSPAERO sweep inputs shared by the generator."""

    api_version: str
    forebody_transition_length_m: float
    aft_taper_start_m: float
    selector_radial_allowance_m: float
    nozzle_radial_allowance_m: float
    fuselage_tessellation: int
    surface_tessellation: int
    lifting_surface_count: int
    lifting_surface_clocking_offset_deg: float
    fin_count: int
    fin_clocking_offset_deg: float
    lifting_surface: SurfacePlanformConfig
    fin: SurfacePlanformConfig
    shell: ExternalShellConfig
    ram_inlet: RamInletConfig
    analysis_method: str
    mach_values: tuple[float, ...]
    alpha_deg_values: tuple[float, ...]
    beta_deg_values: tuple[float, ...]
    reference_cg_x_m: float
    wake_iterations: int

    def __post_init__(self) -> None:
        if not self.api_version.strip():
            raise ValueError("OpenVSP API version cannot be empty")
        for name in (
            "forebody_transition_length_m",
            "aft_taper_start_m",
            "reference_cg_x_m",
        ):
            _positive(name, getattr(self, name))
        for name in ("selector_radial_allowance_m", "nozzle_radial_allowance_m"):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} cannot be negative")
        if self.fuselage_tessellation < 9 or self.surface_tessellation < 4:
            raise ValueError("OpenVSP tessellation settings are too small")
        if self.lifting_surface_count not in (0, 2):
            raise ValueError("lifting surface count must be zero or two")
        if self.fin_count not in (3, 4):
            raise ValueError("fin count must be three or four")
        for name in (
            "lifting_surface_clocking_offset_deg",
            "fin_clocking_offset_deg",
        ):
            if not 0.0 <= getattr(self, name) < 360.0:
                raise ValueError(f"{name} must be in [0, 360) degrees")
        if self.analysis_method not in {"panel", "vortex_lattice"}:
            raise ValueError("analysis method must be 'panel' or 'vortex_lattice'")
        if not self.mach_values or not self.alpha_deg_values or not self.beta_deg_values:
            raise ValueError("VSPAERO sweep arrays cannot be empty")
        if any(value < 0.0 for value in self.mach_values):
            raise ValueError("VSPAERO Mach values cannot be negative")
        if self.wake_iterations < 1:
            raise ValueError("VSPAERO wake iterations must be positive")
        if self.shell.start_x_m < self.forebody_transition_length_m:
            raise ValueError("external shell must start at or aft of the forebody transition")


@dataclass(frozen=True)
class SimulationConfig:
    pulsejet_duration_s: float
    pulsejet_steady_warmup_s: float
    pulsejet_steady_measurement_s: float
    time_step_s: float

    def __post_init__(self) -> None:
        _positive("pulsejet_duration_s", self.pulsejet_duration_s)
        _positive("pulsejet_steady_warmup_s", self.pulsejet_steady_warmup_s)
        _positive(
            "pulsejet_steady_measurement_s",
            self.pulsejet_steady_measurement_s,
        )
        _positive("time_step_s", self.time_step_s)


@dataclass(frozen=True)
class ReferenceCase:
    name: str
    altitude_m: float
    mach: float
    fuel: Fuel
    selector: SelectorConfig
    nozzle: NozzleConfig
    pulsejet: PulsejetConfig
    ramjet: RamjetConfig
    flight: FlightConfig
    vehicle: VehicleConfig
    mission: MissionConfig
    requirements: RequirementsConfig
    geometry: OpenVSPGeometryConfig
    simulation: SimulationConfig

    def __post_init__(self) -> None:
        if self.vehicle.body_diameter_m < self.selector.circular_intake_diameter_m:
            raise ValueError("outer body diameter cannot be smaller than the intake diameter")
        if self.mission.loaded_fuel_mass_kg >= self.flight.initial_mass_kg:
            raise ValueError("loaded fuel mass must be smaller than initial vehicle mass")
        if self.flight.initial_mass_kg > self.requirements.maximum_takeoff_mass_kg:
            raise ValueError("initial vehicle mass exceeds the maximum takeoff mass")
        if self.mission.peak_mach < self.ramjet.minimum_self_sustaining_mach:
            raise ValueError("peak Mach cannot be below the ramjet self-sustaining gate")
        if self.mission.peak_mach <= self.requirements.minimum_peak_mach:
            raise ValueError(
                "mission peak Mach must exceed the competition threshold"
            )
        if self.geometry.forebody_transition_length_m >= self.geometry.aft_taper_start_m:
            raise ValueError("forebody transition must end before the aft taper begins")
        if self.geometry.aft_taper_start_m >= self.vehicle.body_length_m:
            raise ValueError("aft taper must start before the body ends")
        if self.geometry.reference_cg_x_m >= self.vehicle.body_length_m:
            raise ValueError("reference CG must lie within the body length")
        if self.geometry.shell.end_x_m > self.vehicle.body_length_m:
            raise ValueError("external shell must end at or before the body's aft end")
        shell = self.geometry.shell
        shell_constant_span_start_m = shell.start_x_m + shell.forward_taper_length_m
        shell_constant_span_end_m = shell.end_x_m - shell.aft_taper_length_m
        for surface in (self.geometry.lifting_surface, self.geometry.fin):
            if surface.x_location_m + surface.root_chord_m > self.vehicle.body_length_m:
                raise ValueError("surface root chord extends beyond the body length")
            mounts_to_shell = shell.start_x_m <= surface.x_location_m <= shell.end_x_m
            if mounts_to_shell and not (
                shell_constant_span_start_m
                <= surface.x_location_m
                and surface.x_location_m + surface.root_chord_m
                <= shell_constant_span_end_m
            ):
                raise ValueError(
                    "a surface mounted to the external shell must have its entire "
                    "root chord within the shell's constant-diameter span, or its "
                    "root will show a gap against the shell's own taper"
                )
        if self.geometry.lifting_surface_count:
            vspaero_reference_area_m2 = (
                self.geometry.lifting_surface_count
                * self.geometry.lifting_surface.exposed_area_per_surface_m2
            )
            if not isclose(
                self.flight.reference_area_m2,
                vspaero_reference_area_m2,
                rel_tol=1e-9,
                abs_tol=1e-12,
            ):
                raise ValueError(
                    "flight reference area must equal the exposed lifting-surface "
                    "area used by VSPAERO"
                )


def _mapping(data: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = data.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"missing or invalid '{key}' mapping")
    return value


def _read_yaml(path: Path) -> Mapping[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, Mapping):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def load_fuels(fuels_path: str | Path) -> dict[str, Fuel]:
    """Load every provisional fuel entry for transparent comparative trades."""

    fuel_data = _mapping(_read_yaml(Path(fuels_path)), "fuels")
    return {
        str(key): Fuel(key=str(key), **_mapping(fuel_data, str(key)))
        for key in fuel_data
    }


def load_reference_case(
    case_path: str | Path,
    fuels_path: str | Path | None = None,
) -> ReferenceCase:
    case_path = Path(case_path)
    fuels_path = Path(fuels_path) if fuels_path else case_path.with_name("fuels.yaml")
    data = _read_yaml(case_path)
    case_header = _mapping(data, "case")
    fuel_key = str(case_header["fuel_key"])
    fuels = load_fuels(fuels_path)
    if fuel_key not in fuels:
        raise ValueError(f"unknown fuel key: {fuel_key}")

    environment = _mapping(data, "environment")
    selector = _mapping(data, "selector")
    nozzle = _mapping(data, "nozzle")
    pulsejet = _mapping(data, "pulsejet")
    ramjet = _mapping(data, "ramjet")
    flight = _mapping(data, "flight")
    vehicle = _mapping(data, "vehicle")
    mission = _mapping(data, "mission")
    requirements = _mapping(data, "requirements")
    geometry = _mapping(data, "openvsp")
    body_geometry = _mapping(geometry, "body")
    lifting_surface = _mapping(geometry, "lifting_surface")
    fin = _mapping(geometry, "fin")
    shell = _mapping(geometry, "shell")
    ram_inlet = _mapping(geometry, "ram_inlet")
    analysis = _mapping(geometry, "analysis")
    simulation = _mapping(data, "simulation")

    return ReferenceCase(
        name=str(case_header["name"]),
        altitude_m=float(environment["altitude_m"]),
        mach=float(environment["mach"]),
        fuel=fuels[fuel_key],
        selector=SelectorConfig(**selector),
        nozzle=NozzleConfig(**nozzle),
        pulsejet=PulsejetConfig(**pulsejet),
        ramjet=RamjetConfig(**ramjet),
        flight=FlightConfig(**flight),
        vehicle=VehicleConfig(**vehicle),
        mission=MissionConfig(**mission),
        requirements=RequirementsConfig(**requirements),
        geometry=OpenVSPGeometryConfig(
            api_version=str(geometry["api_version"]),
            forebody_transition_length_m=float(
                body_geometry["forebody_transition_length_m"]
            ),
            aft_taper_start_m=float(body_geometry["aft_taper_start_m"]),
            selector_radial_allowance_m=float(
                body_geometry["selector_radial_allowance_m"]
            ),
            nozzle_radial_allowance_m=float(
                body_geometry["nozzle_radial_allowance_m"]
            ),
            fuselage_tessellation=int(body_geometry["fuselage_tessellation"]),
            surface_tessellation=int(geometry["surface_tessellation"]),
            lifting_surface_count=int(geometry["lifting_surface_count"]),
            lifting_surface_clocking_offset_deg=float(
                geometry["lifting_surface_clocking_offset_deg"]
            ),
            fin_count=int(geometry["fin_count"]),
            fin_clocking_offset_deg=float(geometry["fin_clocking_offset_deg"]),
            lifting_surface=SurfacePlanformConfig(**lifting_surface),
            fin=SurfacePlanformConfig(**fin),
            shell=ExternalShellConfig(**shell),
            ram_inlet=RamInletConfig(**ram_inlet),
            analysis_method=str(analysis["method"]),
            mach_values=tuple(float(value) for value in analysis["mach_values"]),
            alpha_deg_values=tuple(
                float(value) for value in analysis["alpha_deg_values"]
            ),
            beta_deg_values=tuple(
                float(value) for value in analysis["beta_deg_values"]
            ),
            reference_cg_x_m=float(analysis["reference_cg_x_m"]),
            wake_iterations=int(analysis["wake_iterations"]),
        ),
        simulation=SimulationConfig(**simulation),
    )
