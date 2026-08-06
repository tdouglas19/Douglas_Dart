"""Generate a JSBSim aircraft model from the same authoritative YAML configuration.

This is the bridge from the repository's own low-order propulsion (`pulsejet.py`,
`ramjet.py`), drag (`drag.py`), and geometry (`config.py`) models into a real 6-DOF
flight-dynamics engine, so trim, short-period, and time-domain behavior can be
checked with JSBSim instead of only the prescribed-flight-path-angle energy-state
model in `trajectory.py`.

Design choices, stated so they are not mistaken for validated aircraft data:

- JSBSim has no native pulsejet or ramjet ``FGEngine`` type. Both are represented as
  ``<external_reactions>`` body-axis forces driven directly by Mach/altitude thrust
  tables built from this repository's own propulsion models (not by a JSBSim engine
  model), gated on or off by a Mach-threshold ``<switch>`` so the two propulsion
  modes stay mutually exclusive, mirroring the physical selector.
- Every coefficient in ``<aerodynamics>`` other than the Mach-indexed zero-lift drag
  (from ``drag.py``) and the configured lift-curve slope is a textbook tail-volume /
  slender-body estimate derived from this vehicle's own fin and lifting-surface
  geometry, not a borrowed or invented constant. They are explicitly flagged
  provisional pending VSPAERO-derived stability derivatives (see
  ``docs/roadmap.md``, "Next: live external-aerodynamics closure").
- No control surfaces are modeled (the configuration does not yet define any); the
  fins and lifting surfaces are fixed stabilizers only. Only static/damping
  derivatives are present, so this model supports open-loop trim and dynamic-mode
  inspection, not closed-loop handling-qualities analysis.
- Mass distribution (`ixx`/`iyy`/`izz`) is a uniform-density solid-cylinder estimate
  from the outer mold line, not a real distributed mass model.
- The fuel tank is present for mass/CG bookkeeping, but nothing drains it during a
  JSBSim run: fuel-limited mission duration remains the job of `trajectory.py`. This
  model is for stability/control/trim inspection at a point, not a full mission burn.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from math import cos, radians, sin
from pathlib import Path
from typing import Any

from .config import ReferenceCase
from .openvsp_geometry import (
    clocking_angles_deg,
    trapezoid_mean_aerodynamic_chord_m,
    vspaero_reference_quantities,
)
from .ramjet import evaluate_ramjet
from .trajectory import (
    ADVERSE_SCENARIO,
    CONSERVATIVE_SCENARIO,
    MissionScenario,
    NOMINAL_SCENARIO,
    _installed_pulsejet_thrust_n,
    _pulsejet_static_thrust_table,
)

N_TO_LBF = 1.0 / 4.4482216152605
G0_M_PER_S2 = 9.80665

_PULSEJET_TABLE_MACH_VALUES = (0.0, 0.10, 0.20, 0.30, 0.40, 0.50)
_PULSEJET_TABLE_ALTITUDES_M = (0.0, 1500.0, 3000.0, 4500.0, 6000.0)
_RAMJET_TABLE_MACH_VALUES = (0.80, 0.90, 1.00, 1.10, 1.20, 1.30)
_RAMJET_TABLE_ALTITUDES_M = (2000.0, 3500.0, 4500.0, 5500.0, 7000.0)


@dataclass(frozen=True)
class StabilityDerivativeEstimate:
    """Geometry-derived, textbook-correlation stability and control derivatives.

    Every value here is computed from this vehicle's own configured fin/lifting-
    surface geometry using standard tail-volume-coefficient formulas. None is fit to
    flight data or a VSPAERO run; all remain provisional until Phase 3 aerodynamic
    closure (`docs/roadmap.md`).
    """

    lift_curve_slope_per_rad: float
    tail_arm_m: float
    horizontal_tail_volume_coefficient: float
    vertical_tail_volume_coefficient: float
    effective_pitch_plane_fin_area_m2: float
    effective_yaw_plane_fin_area_m2: float
    cm_alpha_per_rad: float
    cm_q_per_rad: float
    cn_beta_per_rad: float
    cn_r_per_rad: float
    cl_p_per_rad: float
    cy_beta_per_rad: float
    effective_static_margin_fraction_mac: float
    status: tuple[str, ...]


@dataclass(frozen=True)
class JSBSimAircraftSummary:
    case_name: str
    scenario: str
    output_path: str
    empty_mass_kg: float
    loaded_mass_kg: float
    ixx_kg_m2: float
    iyy_kg_m2: float
    izz_kg_m2: float
    cg_x_m: float
    reference_area_m2: float
    reference_span_m: float
    reference_chord_m: float
    propulsion_mode_switch_mach: float
    pulsejet_table_mach_values: tuple[float, ...]
    pulsejet_table_altitudes_m: tuple[float, ...]
    ramjet_table_mach_values: tuple[float, ...]
    ramjet_table_altitudes_m: tuple[float, ...]
    stability: StabilityDerivativeEstimate
    numerical_reference_only: bool = True


def _effective_fin_plane_areas_m2(case: ReferenceCase) -> tuple[float, float]:
    """Return (pitch-plane, yaw-plane) effective fin area from actual clocking angles.

    Each fin's own lift acts normal to its own plane; the component of that force
    that lands in the vehicle's pitch plane (contributing to Cm) is proportional to
    ``|sin(clocking_angle)|`` and the yaw-plane component to ``|cos(clocking_angle)|``
    for a fin whose root chord lies in a plane through the body axis at that
    clocking angle. Summing over the actual configured fin count and offset (not an
    assumed X- or +-tail constant) keeps this tied to the real geometry.
    """

    angles_deg = clocking_angles_deg(
        case.geometry.fin_count, case.geometry.fin_clocking_offset_deg
    )
    per_fin_area_m2 = case.geometry.fin.exposed_area_per_surface_m2
    pitch_area_m2 = sum(abs(sin(radians(angle))) for angle in angles_deg) * per_fin_area_m2
    yaw_area_m2 = sum(abs(cos(radians(angle))) for angle in angles_deg) * per_fin_area_m2
    return pitch_area_m2, yaw_area_m2


def estimate_stability_derivatives(case: ReferenceCase) -> StabilityDerivativeEstimate:
    """Derive minimal static/damping derivatives from the configured geometry."""

    references = vspaero_reference_quantities(case)
    mean_aerodynamic_chord_m = trapezoid_mean_aerodynamic_chord_m(
        case.geometry.lifting_surface
    )
    fin_aero_center_x_m = (
        case.geometry.fin.x_location_m + 0.25 * case.geometry.fin.root_chord_m
    )
    tail_arm_m = fin_aero_center_x_m - case.geometry.reference_cg_x_m
    if tail_arm_m <= 0.0:
        raise ValueError(
            "fin aerodynamic center must be aft of the reference CG for a "
            "stabilizing tail-volume estimate"
        )

    pitch_area_m2, yaw_area_m2 = _effective_fin_plane_areas_m2(case)
    reference_area_m2 = case.flight.reference_area_m2
    horizontal_tail_volume = (
        pitch_area_m2 * tail_arm_m / (reference_area_m2 * mean_aerodynamic_chord_m)
    )
    vertical_tail_volume = (
        yaw_area_m2 * tail_arm_m / (reference_area_m2 * references.span_m)
    )

    lift_curve_slope_per_rad = case.flight.lift_curve_slope_per_rad
    cm_alpha = -lift_curve_slope_per_rad * horizontal_tail_volume
    cm_q = -2.0 * lift_curve_slope_per_rad * horizontal_tail_volume * (
        tail_arm_m / mean_aerodynamic_chord_m
    )
    cn_beta = lift_curve_slope_per_rad * vertical_tail_volume
    cn_r = -2.0 * lift_curve_slope_per_rad * vertical_tail_volume * (
        tail_arm_m / references.span_m
    )
    # Flat-plate/strip-theory roll-damping approximation for a moderate-AR lifting
    # surface; ignores fin contribution to roll damping entirely.
    cl_p = -lift_curve_slope_per_rad / 4.0
    cy_beta = -lift_curve_slope_per_rad * yaw_area_m2 / reference_area_m2

    static_margin = -cm_alpha / lift_curve_slope_per_rad
    status = [
        "body_and_wing_destabilizing_moment_neglected",
        "tail_downwash_and_sidewash_neglected",
        "roll_damping_is_flat_plate_strip_theory_approximation",
        "no_dihedral_effect_modeled_cl_beta_assumed_zero",
        "pending_vspaero_derived_stability_derivatives",
    ]
    if static_margin < 0.05:
        status.append("effective_static_margin_below_5_percent_mac_marginal")

    return StabilityDerivativeEstimate(
        lift_curve_slope_per_rad=lift_curve_slope_per_rad,
        tail_arm_m=tail_arm_m,
        horizontal_tail_volume_coefficient=horizontal_tail_volume,
        vertical_tail_volume_coefficient=vertical_tail_volume,
        effective_pitch_plane_fin_area_m2=pitch_area_m2,
        effective_yaw_plane_fin_area_m2=yaw_area_m2,
        cm_alpha_per_rad=cm_alpha,
        cm_q_per_rad=cm_q,
        cn_beta_per_rad=cn_beta,
        cn_r_per_rad=cn_r,
        cl_p_per_rad=cl_p,
        cy_beta_per_rad=cy_beta,
        effective_static_margin_fraction_mac=static_margin,
        status=tuple(status),
    )


def _mach_indexed_cd0_breakpoints(case: ReferenceCase) -> list[tuple[float, float]]:
    from .drag import mach_indexed_zero_lift_drag_area_m2

    mach_values = (0.0, 0.40, 0.60, 0.75, 0.85, 0.90, 0.95, 1.00, 1.05, 1.10, 1.20, 1.35, 1.50)
    return [
        (mach, mach_indexed_zero_lift_drag_area_m2(case, mach) / case.flight.reference_area_m2)
        for mach in mach_values
    ]


def _ramjet_thrust_table_n(
    case: ReferenceCase,
    scenario: MissionScenario,
) -> list[list[float]]:
    """Return a [mach][altitude] grid of net ramjet thrust in Newtons."""

    from dataclasses import replace

    recovery = (
        case.selector.ramjet_total_pressure_recovery
        if scenario.ramjet_total_pressure_recovery is None
        else scenario.ramjet_total_pressure_recovery
    )
    selector = replace(case.selector, ramjet_total_pressure_recovery=recovery)
    grid: list[list[float]] = []
    for mach in _RAMJET_TABLE_MACH_VALUES:
        row = []
        for altitude_m in _RAMJET_TABLE_ALTITUDES_M:
            result = evaluate_ramjet(
                case.ramjet, selector, case.nozzle, case.fuel, altitude_m, mach
            )
            row.append(max(result.net_thrust_n, 0.0) * scenario.thrust_multiplier)
        grid.append(row)
    return grid


def _pulsejet_thrust_table_n(
    case: ReferenceCase,
    scenario: MissionScenario,
) -> list[list[float]]:
    """Return a [mach][altitude] grid of net pulsejet thrust in Newtons."""

    sea_level_table = _pulsejet_static_thrust_table(
        case, scenario, _PULSEJET_TABLE_MACH_VALUES
    )
    grid: list[list[float]] = []
    for mach_index, mach in enumerate(_PULSEJET_TABLE_MACH_VALUES):
        row = []
        for altitude_m in _PULSEJET_TABLE_ALTITUDES_M:
            thrust_n, _ = _installed_pulsejet_thrust_n(sea_level_table, mach, altitude_m)
            row.append(max(thrust_n, 0.0))
        grid.append(row)
    return grid


def _el(parent: ET.Element, tag: str, text: Any = None, **attrib: str) -> ET.Element:
    element = ET.SubElement(parent, tag, attrib)
    if text is not None:
        element.text = str(text)
    return element


def _location(parent: ET.Element, unit: str, x: float, y: float, z: float, **attrib: str) -> ET.Element:
    location = _el(parent, "location", unit=unit, **attrib)
    _el(location, "x", f"{x:.6f}")
    _el(location, "y", f"{y:.6f}")
    _el(location, "z", f"{z:.6f}")
    return location


def _table_2d(
    parent: ET.Element,
    row_property: str,
    column_property: str,
    row_values: tuple[float, ...],
    column_values: tuple[float, ...],
    grid_row_major: list[list[float]],
    *,
    row_format: str = "{:.4f}",
    column_format: str = "{:.1f}",
    value_format: str = "{:.4f}",
) -> ET.Element:
    table = _el(parent, "table")
    _el(table, "independentVar", row_property, lookup="row")
    _el(table, "independentVar", column_property, lookup="column")
    lines = ["\t".join([""] + [column_format.format(value) for value in column_values])]
    for row_value, row in zip(row_values, grid_row_major):
        cells = [row_format.format(row_value)] + [value_format.format(value) for value in row]
        lines.append("\t".join(cells))
    data = _el(table, "tableData")
    data.text = "\n" + "\n".join(lines) + "\n"
    return table


def build_jsbsim_aircraft_xml(
    case: ReferenceCase,
    scenario: MissionScenario = NOMINAL_SCENARIO,
) -> tuple[str, JSBSimAircraftSummary]:
    """Return (xml_text, summary) for a JSBSim aircraft model of ``case``."""

    stability = estimate_stability_derivatives(case)
    references = vspaero_reference_quantities(case)
    mean_aerodynamic_chord_m = trapezoid_mean_aerodynamic_chord_m(case.geometry.lifting_surface)

    loaded_mass_kg = case.flight.initial_mass_kg + scenario.mass_growth_kg
    empty_mass_kg = loaded_mass_kg - case.mission.loaded_fuel_mass_kg
    radius_m = 0.5 * case.vehicle.body_diameter_m
    length_m = case.vehicle.body_length_m
    ixx_kg_m2 = 0.5 * loaded_mass_kg * radius_m**2
    iyy_kg_m2 = loaded_mass_kg * (length_m**2 / 12.0 + radius_m**2 / 4.0)
    izz_kg_m2 = iyy_kg_m2
    cg_x_m = case.geometry.reference_cg_x_m

    root = ET.Element(
        "fdm_config",
        {
            "name": f"{case.name}_{scenario.name}",
            "version": "2.0",
            "release": "ALPHA",
        },
    )
    header = _el(root, "fileheader")
    _el(header, "author", "Douglas Dart repository generator (src/douglas_dart/jsbsim_model.py)")
    _el(
        header,
        "description",
        f"Douglas Dart {case.name}, {scenario.name} propulsion/drag scenario. "
        "Auto-generated; not a validated aircraft model.",
    )
    _el(
        header,
        "note",
        "Propulsion is represented entirely via external_reactions thrust tables "
        "built from this repository's own pulsejet/ramjet models, not JSBSim "
        "engine types. Aerodynamic derivatives beyond CD0(Mach) and CLalpha are "
        "textbook tail-volume estimates pending VSPAERO closure. numerical_reference_only.",
    )

    metrics = _el(root, "metrics")
    _el(metrics, "wingarea", f"{references.area_m2:.6f}", unit="M2")
    _el(metrics, "wingspan", f"{references.span_m:.6f}", unit="M")
    _el(metrics, "chord", f"{mean_aerodynamic_chord_m:.6f}", unit="M")
    _el(
        metrics,
        "htailarea",
        f"{stability.effective_pitch_plane_fin_area_m2:.6f}",
        unit="M2",
    )
    _el(metrics, "htailarm", f"{stability.tail_arm_m:.6f}", unit="M")
    _el(
        metrics,
        "vtailarea",
        f"{stability.effective_yaw_plane_fin_area_m2:.6f}",
        unit="M2",
    )
    _el(metrics, "vtailarm", f"{stability.tail_arm_m:.6f}", unit="M")
    _location(metrics, "M", cg_x_m, 0.0, 0.0, name="AERORP")
    _location(metrics, "M", 0.10, 0.0, -0.5 * case.vehicle.body_diameter_m, name="EYEPOINT")
    _location(metrics, "M", cg_x_m, 0.0, 0.0, name="VRP")

    mass_balance = _el(root, "mass_balance")
    _el(mass_balance, "ixx", f"{ixx_kg_m2:.4f}", unit="KG*M2")
    _el(mass_balance, "iyy", f"{iyy_kg_m2:.4f}", unit="KG*M2")
    _el(mass_balance, "izz", f"{izz_kg_m2:.4f}", unit="KG*M2")
    _el(mass_balance, "ixy", "0.0", unit="KG*M2")
    _el(mass_balance, "ixz", "0.0", unit="KG*M2")
    _el(mass_balance, "iyz", "0.0", unit="KG*M2")
    _el(mass_balance, "emptywt", f"{empty_mass_kg:.4f}", unit="KG")
    _location(mass_balance, "M", cg_x_m, 0.0, 0.0, name="CG")

    ground_reactions = _el(root, "ground_reactions")
    contact = _el(ground_reactions, "contact", type="STRUCTURE", name="RECOVERY_SKID")
    _location(contact, "M", cg_x_m, 0.0, radius_m)
    _el(contact, "static_friction", "0.50")
    _el(contact, "dynamic_friction", "0.30")
    _el(contact, "rolling_friction", "0.02")
    _el(contact, "spring_coeff", "150000.0", unit="N/M")
    _el(contact, "damping_coeff", "8000.0", unit="N/M/SEC")
    _el(contact, "max_steer", "0.0", unit="DEG")
    _el(contact, "brake_group", "NONE")
    _el(contact, "retractable", "0")

    pulsejet_grid_n = _pulsejet_thrust_table_n(case, scenario)
    ramjet_grid_n = _ramjet_thrust_table_n(case, scenario)
    pulsejet_grid_lbf = [[value * N_TO_LBF for value in row] for row in pulsejet_grid_n]
    ramjet_grid_lbf = [[value * N_TO_LBF for value in row] for row in ramjet_grid_n]
    transition_mach = case.ramjet.minimum_lightoff_test_mach
    engine_x_m = case.vehicle.body_length_m

    external_reactions = _el(root, "external_reactions")
    _el(external_reactions, "property", "propulsion/pulsejet-active-norm")
    _el(external_reactions, "property", "propulsion/ramjet-active-norm")

    pulsejet_force = _el(external_reactions, "force", name="pulsejet_thrust", frame="BODY")
    pulsejet_function = _el(pulsejet_force, "function")
    pulsejet_product = _el(pulsejet_function, "product")
    _el(pulsejet_product, "property", "propulsion/pulsejet-active-norm")
    _table_2d(
        pulsejet_product,
        "velocities/mach",
        "position/h-sl-meters",
        _PULSEJET_TABLE_MACH_VALUES,
        _PULSEJET_TABLE_ALTITUDES_M,
        pulsejet_grid_lbf,
    )
    _location(pulsejet_force, "M", engine_x_m, 0.0, 0.0)
    direction = _el(pulsejet_force, "direction")
    _el(direction, "x", "1")
    _el(direction, "y", "0")
    _el(direction, "z", "0")

    ramjet_force = _el(external_reactions, "force", name="ramjet_thrust", frame="BODY")
    ramjet_function = _el(ramjet_force, "function")
    ramjet_product = _el(ramjet_function, "product")
    _el(ramjet_product, "property", "propulsion/ramjet-active-norm")
    _table_2d(
        ramjet_product,
        "velocities/mach",
        "position/h-sl-meters",
        _RAMJET_TABLE_MACH_VALUES,
        _RAMJET_TABLE_ALTITUDES_M,
        ramjet_grid_lbf,
    )
    _location(ramjet_force, "M", engine_x_m, 0.0, 0.0)
    direction = _el(ramjet_force, "direction")
    _el(direction, "x", "1")
    _el(direction, "y", "0")
    _el(direction, "z", "0")

    propulsion = _el(root, "propulsion")
    tank = _el(propulsion, "tank", type="FUEL", number="0")
    _location(tank, "M", cg_x_m, 0.0, 0.0)
    _el(tank, "capacity", f"{case.mission.loaded_fuel_mass_kg:.4f}", unit="KG")
    _el(tank, "contents", f"{case.mission.loaded_fuel_mass_kg:.4f}", unit="KG")

    flight_control = _el(root, "flight_control", name="Propulsion Mode Selector")
    channel = _el(flight_control, "channel", name="Intake Selector")
    pulsejet_switch = _el(channel, "switch", name="propulsion/pulsejet-active-norm")
    _el(pulsejet_switch, "default", value="1")
    test = _el(pulsejet_switch, "test", logic="AND", value="0")
    test.text = f"velocities/mach ge {transition_mach:.4f}"
    ramjet_switch = _el(channel, "switch", name="propulsion/ramjet-active-norm")
    _el(ramjet_switch, "default", value="0")
    test = _el(ramjet_switch, "test", logic="AND", value="1")
    test.text = f"velocities/mach ge {transition_mach:.4f}"

    aerodynamics = _el(root, "aerodynamics")

    lift_axis = _el(aerodynamics, "axis", name="LIFT")
    function = _el(lift_axis, "function", name="aero/coefficient/CLalpha")
    _el(function, "description", "Lift_due_to_alpha_configured_lift_curve_slope")
    product = _el(function, "product")
    _el(product, "property", "aero/qbar-psf")
    _el(product, "property", "metrics/Sw-sqft")
    _el(product, "property", "aero/alpha-rad")
    _el(product, "value", f"{stability.lift_curve_slope_per_rad:.4f}")

    drag_axis = _el(aerodynamics, "axis", name="DRAG")
    function = _el(drag_axis, "function", name="aero/coefficient/CD0")
    _el(function, "description", "Mach_indexed_zero_lift_drag_from_drag_py")
    product = _el(function, "product")
    _el(product, "property", "aero/qbar-psf")
    _el(product, "property", "metrics/Sw-sqft")
    table = _el(product, "table")
    _el(table, "independentVar", "velocities/mach", lookup="row")
    data = _el(table, "tableData")
    data.text = "\n" + "\n".join(
        f"{mach:.4f}\t{cd0:.6f}" for mach, cd0 in _mach_indexed_cd0_breakpoints(case)
    ) + "\n"
    function = _el(drag_axis, "function", name="aero/coefficient/CDi")
    _el(function, "description", "Induced_drag_from_configured_induced_drag_factor")
    product = _el(function, "product")
    _el(product, "property", "aero/qbar-psf")
    _el(product, "property", "metrics/Sw-sqft")
    _el(product, "property", "aero/cl-squared")
    _el(product, "value", f"{case.flight.induced_drag_factor:.4f}")

    side_axis = _el(aerodynamics, "axis", name="SIDE")
    function = _el(side_axis, "function", name="aero/coefficient/CYbeta")
    _el(function, "description", "Side_force_due_to_sideslip_from_fin_yaw_plane_area")
    product = _el(function, "product")
    _el(product, "property", "aero/qbar-psf")
    _el(product, "property", "metrics/Sw-sqft")
    _el(product, "property", "aero/beta-rad")
    _el(product, "value", f"{stability.cy_beta_per_rad:.4f}")

    roll_axis = _el(aerodynamics, "axis", name="ROLL")
    function = _el(roll_axis, "function", name="aero/coefficient/Clp")
    _el(function, "description", "Roll_damping_flat_plate_approximation")
    product = _el(function, "product")
    _el(product, "property", "aero/qbar-psf")
    _el(product, "property", "metrics/Sw-sqft")
    _el(product, "property", "metrics/bw-ft")
    _el(product, "property", "aero/bi2vel")
    _el(product, "property", "velocities/p-aero-rad_sec")
    _el(product, "value", f"{stability.cl_p_per_rad:.4f}")

    pitch_axis = _el(aerodynamics, "axis", name="PITCH")
    function = _el(pitch_axis, "function", name="aero/coefficient/Cmalpha")
    _el(function, "description", "Pitch_moment_due_to_alpha_from_horizontal_tail_volume")
    product = _el(function, "product")
    _el(product, "property", "aero/qbar-psf")
    _el(product, "property", "metrics/Sw-sqft")
    _el(product, "property", "metrics/cbarw-ft")
    _el(product, "property", "aero/alpha-rad")
    _el(product, "value", f"{stability.cm_alpha_per_rad:.4f}")
    function = _el(pitch_axis, "function", name="aero/coefficient/Cmq")
    _el(function, "description", "Pitch_damping_from_horizontal_tail_volume_and_arm")
    product = _el(function, "product")
    _el(product, "property", "aero/qbar-psf")
    _el(product, "property", "metrics/Sw-sqft")
    _el(product, "property", "metrics/cbarw-ft")
    _el(product, "property", "aero/ci2vel")
    _el(product, "property", "velocities/q-aero-rad_sec")
    _el(product, "value", f"{stability.cm_q_per_rad:.4f}")

    yaw_axis = _el(aerodynamics, "axis", name="YAW")
    function = _el(yaw_axis, "function", name="aero/coefficient/Cnbeta")
    _el(function, "description", "Yaw_moment_due_to_beta_from_vertical_tail_volume")
    product = _el(function, "product")
    _el(product, "property", "aero/qbar-psf")
    _el(product, "property", "metrics/Sw-sqft")
    _el(product, "property", "metrics/bw-ft")
    _el(product, "property", "aero/beta-rad")
    _el(product, "value", f"{stability.cn_beta_per_rad:.4f}")
    function = _el(yaw_axis, "function", name="aero/coefficient/Cnr")
    _el(function, "description", "Yaw_damping_from_vertical_tail_volume_and_arm")
    product = _el(function, "product")
    _el(product, "property", "aero/qbar-psf")
    _el(product, "property", "metrics/Sw-sqft")
    _el(product, "property", "metrics/bw-ft")
    _el(product, "property", "aero/bi2vel")
    _el(product, "property", "velocities/r-aero-rad_sec")
    _el(product, "value", f"{stability.cn_r_per_rad:.4f}")

    ET.indent(root, space="  ")
    xml_text = (
        '<?xml version="1.0"?>\n'
        + ET.tostring(root, encoding="unicode")
        + "\n"
    )

    summary = JSBSimAircraftSummary(
        case_name=case.name,
        scenario=scenario.name,
        output_path="",
        empty_mass_kg=empty_mass_kg,
        loaded_mass_kg=loaded_mass_kg,
        ixx_kg_m2=ixx_kg_m2,
        iyy_kg_m2=iyy_kg_m2,
        izz_kg_m2=izz_kg_m2,
        cg_x_m=cg_x_m,
        reference_area_m2=references.area_m2,
        reference_span_m=references.span_m,
        reference_chord_m=mean_aerodynamic_chord_m,
        propulsion_mode_switch_mach=transition_mach,
        pulsejet_table_mach_values=_PULSEJET_TABLE_MACH_VALUES,
        pulsejet_table_altitudes_m=_PULSEJET_TABLE_ALTITUDES_M,
        ramjet_table_mach_values=_RAMJET_TABLE_MACH_VALUES,
        ramjet_table_altitudes_m=_RAMJET_TABLE_ALTITUDES_M,
        stability=stability,
    )
    return xml_text, summary


@dataclass(frozen=True)
class JSBSimLoadCheck:
    """Result of actually loading and briefly running the model in real JSBSim."""

    model_name: str
    jsbsim_version: str
    loaded_without_exception: bool
    ran_seconds: float
    final_mach: float
    final_altitude_m: float
    final_alpha_deg: float
    pulsejet_active_at_end: bool
    ramjet_active_at_end: bool
    state_finite: bool
    numerical_reference_only: bool = True


def validate_with_jsbsim(
    aircraft_root_dir: str | Path,
    model_name: str,
    *,
    altitude_m: float,
    mach: float,
    run_seconds: float = 5.0,
) -> JSBSimLoadCheck:
    """Load the generated model in the real JSBSim engine and run it briefly.

    This is genuine verification against the installed ``jsbsim`` package (a hard
    project dependency, not a recording fake): it confirms the XML parses, the
    propulsion-mode switch engages the expected engine, and the state stays finite
    over a short run. It is not a trim, stability, or handling-qualities analysis.
    """

    import math

    import jsbsim as jsbsim_module

    fdm = jsbsim_module.FGFDMExec(str(aircraft_root_dir))
    loaded = fdm.load_model(model_name)
    if not loaded:
        raise RuntimeError(f"JSBSim failed to load model '{model_name}'")

    fdm["ic/h-sl-ft"] = altitude_m / 0.3048
    fdm["ic/mach"] = mach
    fdm.run_ic()
    step_count = int(run_seconds / fdm.get_delta_t())
    for _ in range(max(step_count, 1)):
        fdm.run()

    final_mach = fdm["velocities/mach"]
    final_altitude_m = fdm["position/h-sl-meters"]
    final_alpha_deg = fdm["aero/alpha-deg"]
    state_finite = all(
        math.isfinite(value) for value in (final_mach, final_altitude_m, final_alpha_deg)
    )
    return JSBSimLoadCheck(
        model_name=model_name,
        jsbsim_version=jsbsim_module.FGJSBBase().get_version(),
        loaded_without_exception=True,
        ran_seconds=fdm["simulation/sim-time-sec"],
        final_mach=final_mach,
        final_altitude_m=final_altitude_m,
        final_alpha_deg=final_alpha_deg,
        pulsejet_active_at_end=fdm["propulsion/pulsejet-active-norm"] > 0.5,
        ramjet_active_at_end=fdm["propulsion/ramjet-active-norm"] > 0.5,
        state_finite=state_finite,
    )


_SCENARIOS_BY_NAME = {
    "nominal": NOMINAL_SCENARIO,
    "conservative": CONSERVATIVE_SCENARIO,
    "adverse": ADVERSE_SCENARIO,
}


def write_jsbsim_aircraft(
    case: ReferenceCase,
    output_dir: str | Path,
    scenario: MissionScenario = NOMINAL_SCENARIO,
) -> tuple[Path, JSBSimAircraftSummary]:
    """Write ``<output_dir>/<name>/<name>.xml`` in the JSBSim aircraft-directory layout.

    JSBSim resolves models as ``<root>/aircraft/<name>/<name>.xml``; ``output_dir``
    should be the ``aircraft`` directory of a JSBSim root (e.g. a temp directory or
    ``jsbsim/generated/aircraft``).
    """

    xml_text, summary = build_jsbsim_aircraft_xml(case, scenario)
    model_name = f"{case.name}_{scenario.name}"
    model_dir = Path(output_dir) / model_name
    model_dir.mkdir(parents=True, exist_ok=True)
    output_path = model_dir / f"{model_name}.xml"
    output_path.write_text(xml_text, encoding="utf-8")
    summary = summary.__class__(**{**summary.__dict__, "output_path": str(output_path)})
    return output_path, summary
