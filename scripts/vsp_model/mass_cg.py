"""Mass, CG and inertia for the V4 airframe.

READ THIS BEFORE USING THE NUMBERS. The sizing chain is a POINT-MASS model. It
produces ``mass_budget.csv`` -- a list of items and masses with no stations at
all -- and states outright that CG, moments of inertia and static margin are not
outputs of that repo. Pitching moment out of VSPAERO is meaningless without a
moment reference, and dynamic modes are meaningless without inertias, so this
module builds both.

Every mass here is READ from the sizing export. Every *station* here is
ASSIGNED, either derived from the flowpath geometry (the duct, the skin, the
tank, the fuel -- these have real extents) or chosen outright (avionics, landing
hardware, ballast -- these are packaging decisions nobody has made yet). The
``basis`` field on each item records which, and :func:`describe` prints it.

``payload_ballast_margin`` is 4.68 kg -- 21% of the 22.68 kg wet mass -- with no
assigned location anywhere in the project. That is the single largest CG lever on
the vehicle, and treating it as a solvable variable rather than a fixed lump is
the point of this module.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from math import pi
from pathlib import Path

from geometry_inputs import GeometryInputs
from sizing_input import SizingInput

DEFAULT_MASS_BUDGET_CSV = Path("out_medium_model/v4_export/mass_budget.csv")

# Items that are sums, not parts -- skipped when reading the budget.
_AGGREGATE_ROWS = frozenset({"DRY_MASS", "WET_MASS_AT_RELEASE"})


@dataclass(frozen=True)
class MassItem:
    """One mass with an axial station and enough extent for its own inertia."""

    name: str
    mass_kg: float
    x_m: float
    """Centroid station from the nose tip."""
    length_m: float = 0.0
    """Axial extent, 0 for a lumped item. Used for its own transverse inertia."""
    radius_m: float = 0.0
    """Effective radius of the mass ring, for its own roll inertia."""
    basis: str = ""
    station_source: str = "assigned"
    """``derived`` -- the station follows from flowpath geometry.
    ``chosen``  -- a packaging decision we made, with no model behind it."""

    def transverse_inertia_about_own_cg(self) -> float:
        """I_yy = I_zz of this item about its own centroid (thin shell)."""

        return self.mass_kg * (self.length_m**2 / 12.0 + self.radius_m**2 / 2.0)

    def roll_inertia(self) -> float:
        return self.mass_kg * self.radius_m**2


@dataclass(frozen=True)
class MassProperties:
    items: tuple[MassItem, ...]
    mass_kg: float
    cg_x_m: float
    i_xx_kg_m2: float
    i_yy_kg_m2: float
    i_zz_kg_m2: float
    condition: str

    def cg_fraction_of_body(self, body_length_m: float) -> float:
        return self.cg_x_m / body_length_m


def read_mass_budget(path: str | Path = DEFAULT_MASS_BUDGET_CSV) -> dict[str, float]:
    """Return ``{item: mass_kg}`` from the sizing export, aggregates removed."""

    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"mass budget not found: {path}")
    masses: dict[str, float] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            name = row["item"].strip()
            if name in _AGGREGATE_ROWS:
                continue
            masses[name] = float(row["mass_kg"])
    return masses


def _skin_centroid(sizing: SizingInput, inputs: GeometryInputs) -> tuple[float, float]:
    """Area-weighted centroid and axial extent of the body skin.

    Integrates the derived OML's own surface of revolution rather than assuming
    the skin is a uniform cylinder, so the nose cowl and boattail pull the
    centroid the way they actually do.
    """

    from vehicle_geometry import build_body_stations  # local: avoids a cycle

    stations = build_body_stations(sizing, inputs)
    total_area = 0.0
    moment = 0.0
    for lower, upper in zip(stations, stations[1:]):
        dx = upper.x_m - lower.x_m
        mean_radius = 0.25 * (lower.diameter_m + upper.diameter_m)
        slant = (dx**2 + (upper.radius_m - lower.radius_m) ** 2) ** 0.5
        area = 2.0 * pi * mean_radius * slant
        total_area += area
        moment += area * 0.5 * (lower.x_m + upper.x_m)
    return moment / total_area, stations[-1].x_m - stations[0].x_m


def build_mass_items(
    sizing: SizingInput,
    inputs: GeometryInputs,
    *,
    mass_budget_path: str | Path = DEFAULT_MASS_BUDGET_CSV,
    avionics_x_m: float | None = None,
    landing_hardware_x_m: float | None = None,
    ballast_x_m: float | None = None,
    fuel_fraction: float = 1.0,
) -> tuple[MassItem, ...]:
    """Assign a station to every row of the mass budget.

    ``fuel_fraction`` scales the loaded fuel: 1.0 at release, 0.0 at burnout.
    The tank is an annulus around the tailpipe, well aft of the dry CG, so the
    vehicle's CG moves forward as it burns -- static margin is not one number.
    """

    masses = read_mass_budget(mass_budget_path)
    body_radius = 0.5 * sizing.body_diameter_m
    tailpipe_radius = 0.5 * sizing.nozzle_exit_diameter_m
    chamber_x0, chamber_x1 = sizing.chamber_start_x_m, sizing.chamber_end_x_m
    tailpipe_x0, tailpipe_x1 = sizing.chamber_end_x_m, sizing.boattail_start_x_m
    chamber_length = chamber_x1 - chamber_x0
    tailpipe_length = tailpipe_x1 - tailpipe_x0

    items: list[MassItem] = []

    # --- Engine duct: split chamber/tailpipe by wetted area, exactly the
    #     basis the mass model itself used to compute the total.
    duct_mass = masses["engine_duct_steel"]
    chamber_area = pi * sizing.body_diameter_m * chamber_length
    tailpipe_area = pi * sizing.nozzle_exit_diameter_m * tailpipe_length
    duct_area = chamber_area + tailpipe_area
    items.append(
        MassItem(
            "engine_duct_steel__chamber",
            duct_mass * chamber_area / duct_area,
            0.5 * (chamber_x0 + chamber_x1),
            chamber_length,
            body_radius,
            "steel duct, chamber segment; split from the total by wetted area",
            "derived",
        )
    )
    items.append(
        MassItem(
            "engine_duct_steel__tailpipe",
            duct_mass * tailpipe_area / duct_area,
            0.5 * (tailpipe_x0 + tailpipe_x1),
            tailpipe_length,
            tailpipe_radius,
            "steel duct, tailpipe segment; split from the total by wetted area",
            "derived",
        )
    )

    # --- Skin: distributed over the real OML.
    skin_x, skin_length = _skin_centroid(sizing, inputs)
    items.append(
        MassItem(
            "airframe_skin_cfrp",
            masses["airframe_skin_cfrp"],
            skin_x,
            skin_length,
            body_radius,
            "CFRP skin; centroid integrated over the derived outer mold line",
            "derived",
        )
    )

    # --- Wing: at its own planform centroid, so it tracks wing_root_le_x_m.
    from vehicle_geometry import build_wing_panels  # local: avoids a cycle

    panels = build_wing_panels(sizing, inputs)
    if panels:
        panel = panels[0]
        items.append(
            MassItem(
                "wing",
                masses["wing"],
                panel.quarter_chord_x_m,
                panel.mean_aerodynamic_chord_m,
                # Mid-span radius: the panel mass treated as a ring at its own
                # spanwise centroid, which is what sets its roll inertia.
                panel.mount_radius_m + 0.5 * panel.semispan_m,
                "wing panels lumped at the MAC quarter-chord; moves with wing station",
                "derived",
            )
        )

    # --- Tank hardware and fuel: the annulus around the tailpipe.
    tank_x = 0.5 * (tailpipe_x0 + tailpipe_x1)
    tank_ring_radius = 0.5 * (tailpipe_radius + body_radius)
    items.append(
        MassItem(
            "tank_hardware",
            masses["tank_hardware"],
            tank_x,
            tailpipe_length,
            tank_ring_radius,
            "tank shell; the annulus between tailpipe OD and body ID",
            "derived",
        )
    )
    if fuel_fraction > 0.0:
        items.append(
            MassItem(
                "fuel_loaded",
                masses["fuel_loaded"] * fuel_fraction,
                tank_x,
                tailpipe_length,
                tank_ring_radius,
                f"propane in the tailpipe annulus, {fuel_fraction:.0%} of the loaded mass",
                "derived",
            )
        )

    # --- Chosen stations. Nothing in the project fixes any of these.
    nose_annulus_x = 0.5 * sizing.nose_fairing_length_m
    items.append(
        MassItem(
            "avionics",
            masses["avionics"],
            avionics_x_m if avionics_x_m is not None else nose_annulus_x,
            0.15,
            0.5 * (0.5 * sizing.inlet_lip_diameter_m + body_radius),
            "CHOSEN: packaged in the nose-cowl annulus, forward of the chamber",
            "chosen",
        )
    )
    items.append(
        MassItem(
            "landing_hardware",
            masses["landing_hardware"],
            landing_hardware_x_m
            if landing_hardware_x_m is not None
            else 0.5 * sizing.body_length_m,
            0.0,
            body_radius,
            "CHOSEN: skids/attach points at mid-body",
            "chosen",
        )
    )
    items.append(
        MassItem(
            "payload_ballast_margin",
            masses["payload_ballast_margin"],
            ballast_x_m if ballast_x_m is not None else nose_annulus_x,
            0.20,
            0.5 * body_radius,
            "CHOSEN: 21% of wet mass with no assigned location -- the CG trim lever",
            "chosen",
        )
    )
    return tuple(items)


def mass_properties(items: tuple[MassItem, ...], condition: str) -> MassProperties:
    """Total mass, CG and inertias about the CG."""

    total = sum(item.mass_kg for item in items)
    if total <= 0.0:
        raise ValueError("total mass is not positive")
    cg = sum(item.mass_kg * item.x_m for item in items) / total
    i_transverse = sum(
        item.transverse_inertia_about_own_cg() + item.mass_kg * (item.x_m - cg) ** 2
        for item in items
    )
    i_roll = sum(item.roll_inertia() for item in items)
    return MassProperties(
        items=items,
        mass_kg=total,
        cg_x_m=cg,
        i_xx_kg_m2=i_roll,
        i_yy_kg_m2=i_transverse,
        i_zz_kg_m2=i_transverse,
        condition=condition,
    )


def vehicle_mass_properties(
    sizing: SizingInput,
    inputs: GeometryInputs,
    *,
    fuel_fraction: float = 1.0,
    condition: str | None = None,
    **item_overrides: float | None,
) -> MassProperties:
    items = build_mass_items(
        sizing, inputs, fuel_fraction=fuel_fraction, **item_overrides
    )
    label = condition or (
        "at release (full fuel)" if fuel_fraction >= 1.0 else f"fuel {fuel_fraction:.0%}"
    )
    return mass_properties(items, label)


def describe(properties: MassProperties, body_length_m: float) -> str:
    lines = [f"Mass properties -- {properties.condition}", ""]
    lines.append(f"{'item':<32}{'kg':>8}{'x (mm)':>10}  station")
    for item in sorted(properties.items, key=lambda i: i.x_m):
        lines.append(
            f"{item.name:<32}{item.mass_kg:8.3f}{item.x_m * 1e3:10.1f}  {item.station_source}"
        )
    lines.append("")
    lines.append(f"{'TOTAL':<32}{properties.mass_kg:8.3f}{properties.cg_x_m * 1e3:10.1f}")
    lines.append(
        f"CG at {properties.cg_fraction_of_body(body_length_m):.1%} of body length"
    )
    lines.append(
        f"Ixx {properties.i_xx_kg_m2:.4f}   Iyy {properties.i_yy_kg_m2:.4f}   "
        f"Izz {properties.i_zz_kg_m2:.4f}  kg m^2"
    )
    chosen = [i for i in properties.items if i.station_source == "chosen"]
    if chosen:
        lines.append("")
        lines.append(
            "Stations CHOSEN, not modelled anywhere: "
            + ", ".join(f"{i.name} ({i.mass_kg:.2f} kg)" for i in chosen)
        )
    return "\n".join(lines)
