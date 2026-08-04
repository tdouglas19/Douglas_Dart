# Preliminary fuel trade

## Recommendation

Keep a **Jet-A/JP-8-class kerosene fuel as the baseline**, but do not freeze the exact
grade yet.

The low-order performance differences among Jet-A, gasoline, and liquid propane are
small enough that hardware and safety dominate the choice. Jet-A/JP-8 keeps ambient
liquid storage, avoids propane's pressure-vessel penalty, retains much more modeled
endurance than ethanol, and starts from established aviation-turbine fuel
specifications. The active U.S. JP-8 specification is MIL-DTL-83133, and FAA guidance
identifies ASTM D1655 as the primary Jet-A/Jet-A-1 specification.

This is a system-level recommendation, not proof that kerosene will light or sustain
in the pulsejet/ramjet combustor. Atomization, igniter energy, flameholding, cold/hot
restart, and transition must be demonstrated.

## Performance-only comparison

The command

```bash
douglas-dart fuel-trade --config configs/shared_nozzle_candidate_b.yaml
```

uses the provisional properties in `configs/fuels.yaml`. It holds geometry,
temperature targets, fuel mass allocations, and every non-fuel parameter fixed.

| Fuel reference | Volume for 3.8 kg | Chemical energy | Pulsejet steady net | Ramjet net | Static hold |
|---|---:|---:|---:|---:|---:|
| Jet-A | 4.75 L | 163.4 MJ | 122.0 N | 832.1 N | 26.2 s |
| Gasoline | 5.14 L | 167.2 MJ | 126.0 N | 831.5 N | 26.8 s |
| Liquid propane | 7.71 L | 176.3 MJ | 125.5 N | 830.2 N | 28.2 s |
| Ethanol | 4.82 L | 101.8 MJ | 128.5 N | 848.2 N | 16.6 s |

Ethanol's higher modeled ramjet net thrust is not a free benefit. At a fixed nozzle
capacity and target combustor temperature, its higher required fuel fraction reduces
airflow and inlet momentum drag; its much higher fuel flow then cuts endurance. This
is one reason thrust alone is a poor fuel-selection metric.

The U.S. Department of Energy comparison likewise reports lower volumetric energy for
propane and ethanol than gasoline and identifies propane as a pressurized liquid.
Those published relationships support the direction of the storage trade, but the
repository values remain representative rather than specification-certified.

## Criteria not yet scored

FAA fuel-approval guidance explicitly calls for evaluating density, energy content,
combustion behavior, starting/restarting, material compatibility, vapor pressure,
flammability/flash point, water/icing, viscosity, and pump pressure drop. Candidate B
also needs criteria specific to this architecture:

- pulsejet pressure-cycle repeatability and acoustic coupling;
- ramjet light-off and flameholding at Mach 0.8–1.1;
- atomizer droplet size over the required turndown;
- selector-transition ignition and trapped-volume behavior;
- tank, pump, regulator, injector, purge, and fire-protection mass;
- thermal soak and vapor formation;
- ground handling and range/test-cell controls; and
- fuel availability and competition restrictions.

No automatic weighted score is implemented because those values and weights are not
closed. The code reports performance and storage-volume facts, then leaves the safety
and hardware decision visible.

## Sources to use for closure

- [U.S. DOE Alternative Fuels Data Center fuel-property comparison](https://afdc.energy.gov/fuels/properties)
- [FAA AC 20-24D, propulsion fuel approval and evaluation
  guidance](https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC_20-24D.pdf)
- [DLA ASSIST current MIL-DTL-83133 JP-8 specification
  record](https://quicksearch.dla.mil/qsDocDetails.aspx?ident_number=33505)

Before hardware sizing, replace each representative value with a documented range
from the exact procured grade and propagate that range through fuel flow, tank volume,
CG, and mission endurance.
