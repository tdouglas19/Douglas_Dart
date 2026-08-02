# OpenVSP/VSPAERO integration

This directory will hold source scripts and small templates. Generated `.vsp3`
models, meshes, and polars belong under `openvsp/generated/` and are ignored by Git.

The planned interface is:

1. Read the same versioned vehicle/propulsion geometry configuration used by Python.
2. Generate body, intake reference geometry, fins, and lifting surfaces through the
   OpenVSP API.
3. save a `.vsp3` artifact with configuration hash and OpenVSP version metadata;
4. run VSPAERO Mach/angle/sideslip sweeps headlessly; and
5. normalize coefficients and reference dimensions into a table consumed by the
   flight model.

No geometry generator is committed yet because the OpenVSP version and local API
module have not been pinned or verified in this environment. That is an explicit
dependency gate, not an invitation to add an untested script.
