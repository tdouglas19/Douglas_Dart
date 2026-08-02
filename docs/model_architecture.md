# Model architecture

## Discipline boundary

The propulsion and flight models are connected through explicit, inspectable
interfaces rather than one monolithic simulation.

```mermaid
flowchart TD
    C["Versioned YAML inputs"] --> P["Pulsejet / ramjet models"]
    C --> G["OpenVSP geometry generator"]
    G --> A["VSPAERO aerodynamic tables"]
    P --> F["Point-mass flight model"]
    A --> F
```

VSPAERO will provide external aerodynamic forces, moments, and stability data. It
will not be used for combustor or nozzle reacting flow.

## Intake selector

The user requirement is encoded directly:

\[
A_{available}=f_{open}A_{circular},\qquad f_{open}=0.5
\]

The flow restriction then applies a separate discharge coefficient. This prevents
geometric blockage from being silently combined with loss calibration. Only one
mode can be selected at a time by `DualModePropulsion`.

## Pulsejet state and sequence

The chamber is a well-stirred, constant-volume control volume. Its dynamic state is
total mass, fresh-air mass, unburned-fuel mass, internal energy, and pending
chemical heat release. The step sequence is:

1. Check whether pressure, refill mass, and minimum-period ignition criteria are met.
2. Schedule a finite-duration heat release for the burnable fuel/air charge.
3. Calculate forward inlet flow from the recovered freestream stagnation reservoir.
4. Calculate nozzle blowdown from instantaneous chamber pressure and temperature.
5. Apply inlet/outlet enthalpy transport, combustion heat release, and wall loss.
6. Recover pressure from the ideal-gas constant-volume relation.

The governing lumped energy relation is

\[
\frac{dU}{dt}=\dot m_{in}h_{in}-\dot m_{out}h_{out}+\dot Q_{comb}-\dot Q_{wall}.
\]

This provides the requested rise, decay, pressure-differential intake, combustion,
and repeat behavior. It does not prove that an acoustic mode will sustain itself;
that requires calibration or a higher-order gas-dynamics model.

## Fixed C-D nozzle

The nozzle model checks the critical pressure ratio. Below choking, it treats the
throat as a compressible restriction and expands to ambient. When choked, it solves
the supersonic area-Mach branch for the specified exit/throat area ratio and uses

\[
F_g=\dot m V_e+(p_e-p_a)A_e.
\]

Strong overexpansion is flagged because internal shocks, separation, hysteresis,
and transient wave interaction are not represented.

## Ramjet

The steady model calculates captured air mass flow, recovered total pressure,
combustor pressure loss, fuel/air ratio to reach a specified exit temperature, and
fixed-nozzle performance. It reports the mismatch between captured combustor flow
and the nozzle's capacity. When the nozzle is undersized, excess potential capture
is treated as inlet spillage and only nozzle-compatible flow contributes to thrust.
That residual must be driven close to zero through geometry/operating-point
iteration before the point is credible.

The minimum self-sustaining Mach number is an explicit configuration gate. A point
below it may still be calculated for continuity, but it is never labeled an
operable design point.

## Flight dynamics

The initial flight kernel is two-dimensional and point-mass. It preserves speed,
flight-path angle, altitude, downrange, and vehicle mass. The current parabolic drag
polar is only a temporary surrogate; it will be replaced by Mach/angle tables from
VSPAERO before trajectory conclusions are drawn.
